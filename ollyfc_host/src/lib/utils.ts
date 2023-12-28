export function hasNull(obj: any) {
  for (let key in obj) {
    if (obj[key] === null) {
      return true; // Found a null leaf
    } else if (typeof obj[key] === 'object' && obj[key] !== null) {
      // Recursively check nested objects
      if (hasNull(obj[key])) {
        return true;
      }
    }
  }
  return false;
}