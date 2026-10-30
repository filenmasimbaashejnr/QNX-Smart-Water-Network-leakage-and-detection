/*
 * leakmon2.c - Smart Water Network Leakage Detection, Master Pi 2 (QNX Neutrino)
 *
 * One ESP32 slave (Esp32_single.INO) with three flow sensors and two valves:
 *
 *   pump -- F1 --[V1]-- F2 --[V2]-- F3 -- demand
 *           \______ ESP32 0x32 (Esp32_single.INO) ______/
 *
 *   F1 > F2 (beyond tolerance, confirmed) -> leak in seg1 -> close V1 (nearest)
 *   F2 > F3 (beyond tolerance, confirmed) -> leak in seg2 -> close V2 (nearest)
 *
 * Threads
 *   main      : SIGINT/SIGTERM = quit, SIGUSR1 = clear both latches + reopen V1 and V2
 *   poll      : the ONLY thread that touches I2C
 *   monitor   : leak localisation, never touches I2C
 *
 * ESP32 READ  10 bytes :
 *   [0xA5][F1 L][F1 H][F2 L][F2 H][F3 L][F3 H][V1][V2][XOR of bytes 0..8]
 * Master WRITE 4 bytes :
 *   [0x10][valve 0=V1 1=V2][arg 0=close 1=open][0x10 ^ valve ^ arg ^ 0x5A]
 *
 * Build:  qcc -Vgcc_ntoaarch64le -o leakmon2 MasterPi2.c
 * Run:    ./leakmon2 -d /dev/i2c1 -a 0x32 -p 500 -t 200 -r 5 -n 3
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <time.h>
#include <signal.h>
#include <unistd.h>
#include <fcntl.h>
#include <pthread.h>
#include <sched.h>
#include <devctl.h>
#include <hw/i2c.h>
#include <sys/neutrino.h>

#define PKT_HDR        0xA5
#define PKT_LEN        10
#define CMD_VALVE      0x10
#define CMD_XOR_SEED   0x5A

#define POLL_PRIO      21
#define MONITOR_PRIO   20

#define N_FLOW         3
#define N_SEG          2           /* seg1 = F1-F2 / V1,  seg2 = F2-F3 / V2 */

/* ------------------------------------------------------------------ types */

typedef struct {
    const char *dev;
    uint8_t     addr;               /* Esp32_single */
    unsigned    period_ms;
    unsigned    i2c_timeout_ms;
    unsigned    retries;
    unsigned    stale_ms;
    double      tol_abs_ml;
    double      tol_pct;
    double      min_flow_ml;
    unsigned    confirm;
} config_t;

typedef enum { CMD_NONE = 0, CMD_VALVE_CLOSE, CMD_VALVE_OPEN } valve_cmd_t;

typedef struct {
    bool     valid;
    bool     valve_open[N_SEG];
    uint64_t stamp_ms;
    unsigned consec_err;
} esp_t;

typedef struct {
    pthread_mutex_t lock;
    pthread_cond_t  data_cv;
    pthread_cond_t  cmd_cv;
    esp_t           esp;
    uint16_t        flow_ml_min[N_FLOW];   /* F1, F2, F3 */
    uint64_t        seq;
    valve_cmd_t     cmd[N_SEG];            /* pending V1 / V2 */
    bool            leak_latched[N_SEG];
    bool            quit;
} shared_t;

typedef struct {
    config_t cfg;
    shared_t sh;
    int      fd;
} app_t;

/* ---------------------------------------------------------------- helpers */

static uint64_t now_ms(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (uint64_t)t.tv_sec * 1000u + (uint64_t)t.tv_nsec / 1000000u;
}

static void ts_add_ms(struct timespec *t, unsigned ms)
{
    t->tv_sec  += ms / 1000u;
    t->tv_nsec += (long)(ms % 1000u) * 1000000L;
    if (t->tv_nsec >= 1000000000L) {
        t->tv_sec++;
        t->tv_nsec -= 1000000000L;
    }
}

static bool ts_before(const struct timespec *a, const struct timespec *b)
{
    return a->tv_sec < b->tv_sec ||
          (a->tv_sec == b->tv_sec && a->tv_nsec < b->tv_nsec);
}

#define LOG(fmt, ...) \
    do { printf("[%8llu ms] " fmt "\n", (unsigned long long)now_ms(), ##__VA_ARGS__); fflush(stdout); } while (0)

/* ------------------------------------------------------- I2C (QNX devctl) */

static int devctl_timed(int fd, int dcmd, void *data, size_t nbytes, unsigned timeout_ms)
{
    struct sigevent ev;
    uint64_t t = (uint64_t)timeout_ms * 1000000ull;

    SIGEV_UNBLOCK_INIT(&ev);
    TimerTimeout(CLOCK_MONOTONIC, _NTO_TIMEOUT_SEND | _NTO_TIMEOUT_REPLY, &ev, &t, NULL);
    return devctl(fd, dcmd, data, nbytes, NULL);
}

static int i2c_write(int fd, uint8_t addr, const uint8_t *buf, size_t len, unsigned to_ms)
{
    struct { i2c_send_t hdr; uint8_t data[8]; } m;

    if (len > sizeof m.data) return EINVAL;
    memset(&m, 0, sizeof m);
    m.hdr.slave.addr = addr;
    m.hdr.slave.fmt  = I2C_ADDRFMT_7BIT;
    m.hdr.len        = (uint32_t)len;
    m.hdr.stop       = 1;
    memcpy(m.data, buf, len);
    return devctl_timed(fd, DCMD_I2C_SEND, &m, sizeof m.hdr + len, to_ms);
}

static int i2c_read(int fd, uint8_t addr, uint8_t *buf, size_t len, unsigned to_ms)
{
    struct { i2c_recv_t hdr; uint8_t data[16]; } m;
    int rc;

    if (len > sizeof m.data) return EINVAL;
    memset(&m, 0, sizeof m);
    m.hdr.slave.addr = addr;
    m.hdr.slave.fmt  = I2C_ADDRFMT_7BIT;
    m.hdr.len        = (uint32_t)len;
    m.hdr.stop       = 1;
    rc = devctl_timed(fd, DCMD_I2C_RECV, &m, sizeof m.hdr + len, to_ms);
    if (rc == EOK) memcpy(buf, m.data, len);
    return rc;
}

static uint8_t xor8(const uint8_t *p, size_t n)
{
    uint8_t x = 0;
    while (n--) x ^= *p++;
    return x;
}

static int node_read(const config_t *c, int fd,
                     uint16_t flow[N_FLOW], bool valve[N_SEG])
{
    uint8_t buf[PKT_LEN];
    int rc = EIO;

    for (unsigned a = 0; a <= c->retries; a++) {
        rc = i2c_read(fd, c->addr, buf, PKT_LEN, c->i2c_timeout_ms);
        if (rc == EOK) {
            if (buf[0] != PKT_HDR || xor8(buf, 9) != buf[9]) {
                rc = EBADMSG;
            } else {
                flow[0]  = (uint16_t)(buf[1] | (buf[2] << 8));
                flow[1]  = (uint16_t)(buf[3] | (buf[4] << 8));
                flow[2]  = (uint16_t)(buf[5] | (buf[6] << 8));
                valve[0] = buf[7] != 0;
                valve[1] = buf[8] != 0;
                return EOK;
            }
        }
        usleep(5000);
    }
    return rc;
}

static int valve_command(const config_t *c, int fd, int which, valve_cmd_t cmd)
{
    uint8_t arg = (cmd == CMD_VALVE_OPEN) ? 1 : 0;
    uint8_t vid = (uint8_t)which;
    uint8_t pkt[4] = {
        CMD_VALVE, vid, arg,
        (uint8_t)(CMD_VALVE ^ vid ^ arg ^ CMD_XOR_SEED)
    };
    int rc = EIO;

    for (unsigned a = 0; a <= c->retries + 1; a++) {
        rc = i2c_write(fd, c->addr, pkt, sizeof pkt, c->i2c_timeout_ms);
        if (rc == EOK) {
            uint16_t fl[N_FLOW] = { 0 };
            bool v[N_SEG] = { false, false };
            usleep(50000);
            rc = node_read(c, fd, fl, v);
            if (rc == EOK) {
                if (v[which] == (arg != 0)) return EOK;
                rc = EPROTO;
            }
        }
        usleep(20000);
    }
    return rc;
}

/* ----------------------------------------------------------- poll thread */

static void poll_all(app_t *app)
{
    const config_t *c = &app->cfg;
    shared_t *s = &app->sh;
    uint16_t fl[N_FLOW] = { 0 };
    bool v[N_SEG] = { true, true };
    int rc;

    rc = node_read(c, app->fd, fl, v);

    pthread_mutex_lock(&s->lock);

    if (rc == EOK) {
        s->esp.valid = true;
        s->esp.valve_open[0] = v[0];
        s->esp.valve_open[1] = v[1];
        s->esp.stamp_ms = now_ms();
        s->esp.consec_err = 0;
        memcpy(s->flow_ml_min, fl, sizeof fl);
    } else {
        if (++s->esp.consec_err == 1 || s->esp.consec_err % 20 == 0)
            LOG("ESP32 (0x%02X) read failed: %s (consecutive errors: %u)",
                c->addr, strerror(rc), s->esp.consec_err);
    }

    s->seq++;
    pthread_cond_broadcast(&s->data_cv);
    pthread_mutex_unlock(&s->lock);
}

static void exec_valve(app_t *app, int which, valve_cmd_t cmd)
{
    const char *name = (which == 0) ? "V1 (seg1, nearest F1-F2)" : "V2 (seg2, nearest F2-F3)";
    int rc = valve_command(&app->cfg, app->fd, which, cmd);

    if (rc == EOK) {
        LOG("%s %s confirmed", name, cmd == CMD_VALVE_CLOSE ? "CLOSED" : "OPENED");
    } else {
        LOG("!! %s %s FAILED: %s - retrying next cycle",
            name, cmd == CMD_VALVE_CLOSE ? "close" : "open", strerror(rc));
        pthread_mutex_lock(&app->sh.lock);
        if (app->sh.cmd[which] == CMD_NONE) app->sh.cmd[which] = cmd;
        pthread_mutex_unlock(&app->sh.lock);
    }
}

static void *poll_thread(void *arg)
{
    app_t *app = arg;
    shared_t *s = &app->sh;
    struct timespec next, now;

    clock_gettime(CLOCK_MONOTONIC, &next);

    for (;;) {
        valve_cmd_t cmd[N_SEG];

        poll_all(app);

        ts_add_ms(&next, app->cfg.period_ms);
        clock_gettime(CLOCK_MONOTONIC, &now);
        if (ts_before(&next, &now)) next = now;

        pthread_mutex_lock(&s->lock);
        while (!s->quit && s->cmd[0] == CMD_NONE && s->cmd[1] == CMD_NONE) {
            if (pthread_cond_timedwait(&s->cmd_cv, &s->lock, &next) == ETIMEDOUT) break;
        }
        if (s->quit) { pthread_mutex_unlock(&s->lock); break; }
        cmd[0] = s->cmd[0]; s->cmd[0] = CMD_NONE;
        cmd[1] = s->cmd[1]; s->cmd[1] = CMD_NONE;
        pthread_mutex_unlock(&s->lock);

        if (cmd[0] != CMD_NONE) exec_valve(app, 0, cmd[0]);
        if (cmd[1] != CMD_NONE) exec_valve(app, 1, cmd[1]);
    }
    return NULL;
}

/* -------------------------------------------------------- monitor thread */

static bool segment_leak(double up, double down, double min_flow,
                         double tol_abs, double tol_pct, unsigned *over,
                         unsigned confirm, const char *mismatch_tag)
{
    if (up < min_flow && down < min_flow) {
        *over = 0;
        return false;
    }

    double tol  = tol_abs + (tol_pct / 100.0) * up;
    double diff = up - down;

    if (diff > tol) {
        if (++(*over) >= confirm) return true;
        return false;
    }

    if (-diff > tol)
        LOG("warn: %s exceeds upstream by more than tolerance (sensor mismatch?)", mismatch_tag);
    *over = 0;
    return false;
}

static void *monitor_thread(void *arg)
{
    app_t *app = arg;
    const config_t *c = &app->cfg;
    shared_t *s = &app->sh;
    uint64_t last_seq = 0;
    unsigned over[N_SEG] = { 0, 0 };

    pthread_mutex_lock(&s->lock);
    for (;;) {
        esp_t e;
        uint16_t fl[N_FLOW];
        bool latched[N_SEG];

        while (!s->quit && s->seq == last_seq) {
            struct timespec dl;
            clock_gettime(CLOCK_MONOTONIC, &dl);
            ts_add_ms(&dl, c->period_ms * 3);
            if (pthread_cond_timedwait(&s->data_cv, &s->lock, &dl) == ETIMEDOUT)
                LOG("!! monitor: no new samples from poll thread");
        }
        if (s->quit) break;

        last_seq = s->seq;
        e = s->esp;
        memcpy(fl, s->flow_ml_min, sizeof fl);
        latched[0] = s->leak_latched[0];
        latched[1] = s->leak_latched[1];
        pthread_mutex_unlock(&s->lock);

        bool leak[N_SEG] = { false, false };
        uint64_t t = now_ms();
        bool fresh = e.valid && (t - e.stamp_ms) <= c->stale_ms;

        if (!fresh) {
            over[0] = over[1] = 0;
        } else {
            double f1 = fl[0], f2 = fl[1], f3 = fl[2];

            LOG("F1=%.3f  F2=%.3f  F3=%.3f L/min  V1=%s  V2=%s%s%s",
                f1 / 1000.0, f2 / 1000.0, f3 / 1000.0,
                e.valve_open[0] ? "OPEN" : "CLOSED",
                e.valve_open[1] ? "OPEN" : "CLOSED",
                latched[0] ? "  [SEG1 LATCHED]" : "",
                latched[1] ? "  [SEG2 LATCHED]" : "");

            leak[0] = segment_leak(f1, f2, c->min_flow_ml, c->tol_abs_ml, c->tol_pct,
                                   &over[0], c->confirm, "Flow 2");
            leak[1] = segment_leak(f2, f3, c->min_flow_ml, c->tol_abs_ml, c->tol_pct,
                                   &over[1], c->confirm, "Flow 3");
        }

        pthread_mutex_lock(&s->lock);
        if (leak[0] && !s->leak_latched[0]) {
            s->leak_latched[0] = true;
            s->cmd[0] = CMD_VALVE_CLOSE;
            pthread_cond_signal(&s->cmd_cv);
            LOG("*** LEAK in segment 1 (F1-F2, %u cycles) - closing V1 (nearest) ***", over[0]);
        }
        if (leak[1] && !s->leak_latched[1]) {
            s->leak_latched[1] = true;
            s->cmd[1] = CMD_VALVE_CLOSE;
            pthread_cond_signal(&s->cmd_cv);
            LOG("*** LEAK in segment 2 (F2-F3, %u cycles) - closing V2 (nearest) ***", over[1]);
        }
    }
    pthread_mutex_unlock(&s->lock);
    return NULL;
}

/* ------------------------------------------------------------------ main */

static int spawn(pthread_t *t, void *(*fn)(void *), void *arg, int prio, const char *name)
{
    pthread_attr_t at;
    struct sched_param sp = { .sched_priority = prio };
    int rc;

    pthread_attr_init(&at);
    pthread_attr_setinheritsched(&at, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy(&at, SCHED_RR);
    pthread_attr_setschedparam(&at, &sp);
    rc = pthread_create(t, &at, fn, arg);
    pthread_attr_destroy(&at);
    if (rc != EOK) rc = pthread_create(t, NULL, fn, arg);
    if (rc == EOK) pthread_setname_np(*t, name);
    return rc;
}

static void usage(const char *p)
{
    fprintf(stderr,
        "usage: %s [-d dev] [-a addr] [-p period_ms] [-o i2c_timeout_ms]\n"
        "          [-t tol_abs_mL_min] [-r tol_pct] [-m min_flow_mL_min] [-n confirm_cycles] [-s stale_ms]\n", p);
}

int main(int argc, char **argv)
{
    static app_t app;
    config_t *c = &app.cfg;
    shared_t *s = &app.sh;
    pthread_t th_poll, th_mon;
    pthread_mutexattr_t ma;
    pthread_condattr_t ca;
    sigset_t sigs;
    int opt;

    c->dev = "/dev/i2c1";
    c->addr = 0x32;
    c->period_ms = 500;  c->i2c_timeout_ms = 100; c->retries = 2; c->stale_ms = 2000;
    c->tol_abs_ml = 200.0; c->tol_pct = 5.0; c->min_flow_ml = 100.0; c->confirm = 3;

    while ((opt = getopt(argc, argv, "d:a:p:o:t:r:m:n:s:h")) != -1) {
        switch (opt) {
        case 'd': c->dev = optarg; break;
        case 'a': c->addr = (uint8_t)strtoul(optarg, NULL, 0); break;
        case 'p': c->period_ms = (unsigned)strtoul(optarg, NULL, 0); break;
        case 'o': c->i2c_timeout_ms = (unsigned)strtoul(optarg, NULL, 0); break;
        case 't': c->tol_abs_ml = strtod(optarg, NULL); break;
        case 'r': c->tol_pct = strtod(optarg, NULL); break;
        case 'm': c->min_flow_ml = strtod(optarg, NULL); break;
        case 'n': c->confirm = (unsigned)strtoul(optarg, NULL, 0); break;
        case 's': c->stale_ms = (unsigned)strtoul(optarg, NULL, 0); break;
        default:  usage(argv[0]); return EXIT_FAILURE;
        }
    }
    if (c->period_ms < 50 || c->confirm < 1) { usage(argv[0]); return EXIT_FAILURE; }

    app.fd = open(c->dev, O_RDWR);
    if (app.fd < 0) { perror("open i2c device (is i2c-bcm2711 running?)"); return EXIT_FAILURE; }
    {
        uint32_t speed = 100000;
        if (devctl(app.fd, DCMD_I2C_SET_BUS_SPEED, &speed, sizeof speed, NULL) != EOK)
            LOG("warn: could not set I2C bus speed (driver default used)");
    }

    pthread_mutexattr_init(&ma);
    pthread_mutex_init(&s->lock, &ma);
    pthread_condattr_init(&ca);
    pthread_condattr_setclock(&ca, CLOCK_MONOTONIC);
    pthread_cond_init(&s->data_cv, &ca);
    pthread_cond_init(&s->cmd_cv, &ca);

    sigemptyset(&sigs);
    sigaddset(&sigs, SIGINT); sigaddset(&sigs, SIGTERM); sigaddset(&sigs, SIGUSR1);
    pthread_sigmask(SIG_BLOCK, &sigs, NULL);

    LOG("leakmon2: %s  ESP32=0x%02X (F1+F2+F3, V1+V2)  period=%ums  tol=%.0f mL/min + %.1f%%  confirm=%u",
        c->dev, c->addr, c->period_ms, c->tol_abs_ml, c->tol_pct, c->confirm);

    if (spawn(&th_poll, poll_thread, &app, POLL_PRIO, "i2c_poll") != EOK ||
        spawn(&th_mon, monitor_thread, &app, MONITOR_PRIO, "leak_monitor") != EOK) {
        fprintf(stderr, "thread creation failed\n");
        return EXIT_FAILURE;
    }

    for (;;) {
        int sig;
        sigwait(&sigs, &sig);
        if (sig == SIGUSR1) {
            pthread_mutex_lock(&s->lock);
            s->leak_latched[0] = false;
            s->leak_latched[1] = false;
            s->cmd[0] = CMD_VALVE_OPEN;
            s->cmd[1] = CMD_VALVE_OPEN;
            pthread_cond_signal(&s->cmd_cv);
            pthread_mutex_unlock(&s->lock);
            LOG("operator reset: both latches cleared, reopening V1 and V2");
        } else {
            break;
        }
    }

    pthread_mutex_lock(&s->lock);
    s->quit = true;
    pthread_cond_broadcast(&s->data_cv);
    pthread_cond_broadcast(&s->cmd_cv);
    pthread_mutex_unlock(&s->lock);

    pthread_join(th_poll, NULL);
    pthread_join(th_mon, NULL);
    close(app.fd);
    LOG("leakmon2: shut down (valves left in last state)");
    return EXIT_SUCCESS;
}
