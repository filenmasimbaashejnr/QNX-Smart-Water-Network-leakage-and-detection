export const auth = getAuth(app);
export const database = getDatabase(app);

export function loginUser(email, password) {
  return signInWithEmailAndPassword(auth, email.trim(), password);
}

export function logoutUser() {
  return signOut(auth);
}