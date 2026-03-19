//! Design parameter storage for parameterized solids.
//!
//! [`ParamStore`] holds named scalar design variables that can be referenced
//! from [`FieldNode`](crate::field_node::FieldNode) variants via
//! [`Val::Param`](crate::field_node::Val). Values are updated between
//! evaluations via [`ParamStore::set`]; reads during evaluation go through
//! an `RwLock` that allows concurrent readers (parallel meshing via rayon).
//!
//! Session 25 will extend this with `∂f/∂param_i` gradient computation.

use std::sync::{Arc, PoisonError, RwLock, RwLockReadGuard, RwLockWriteGuard};

/// Design parameter store — create, name, and update scalar design variables.
///
/// Typical usage:
/// ```
/// use cf_design::{ParamStore, Solid};
///
/// let store = ParamStore::new();
/// let radius = store.add("radius", 5.0);
/// let s = Solid::sphere_p(radius);
///
/// store.set("radius", 10.0);
/// // Next evaluate() call uses the updated radius.
/// ```
///
/// Clone is cheap (shared `Arc` reference).
#[derive(Clone, Debug)]
pub struct ParamStore(pub(crate) Arc<ParamStoreData>);

#[derive(Debug, Default)]
pub struct ParamStoreData {
    inner: RwLock<ParamEntries>,
}

#[derive(Debug, Default)]
struct ParamEntries {
    names: Vec<String>,
    values: Vec<f64>,
}

/// A reference to a named design parameter inside a [`ParamStore`].
///
/// Returned by [`ParamStore::add`]. Pass to `_p` constructors on [`Solid`](crate::Solid)
/// (e.g., [`Solid::sphere_p`](crate::Solid::sphere_p)) to create parameterized geometry.
#[derive(Clone, Debug)]
pub struct ParamRef {
    pub(crate) id: usize,
    pub(crate) store: Arc<ParamStoreData>,
}

impl ParamRef {
    /// The current value of this parameter.
    #[must_use]
    pub fn value(&self) -> f64 {
        self.store.get_by_id(self.id)
    }
}

/// Recover from a poisoned read lock.
fn read(lock: &RwLock<ParamEntries>) -> RwLockReadGuard<'_, ParamEntries> {
    lock.read().unwrap_or_else(PoisonError::into_inner)
}

/// Recover from a poisoned write lock.
fn write(lock: &RwLock<ParamEntries>) -> RwLockWriteGuard<'_, ParamEntries> {
    lock.write().unwrap_or_else(PoisonError::into_inner)
}

impl ParamStore {
    /// Create an empty parameter store.
    #[must_use]
    pub fn new() -> Self {
        Self(Arc::new(ParamStoreData::default()))
    }

    /// Register a named parameter with an initial value.
    ///
    /// Returns a [`ParamRef`] for use in parameterized `Solid` constructors.
    ///
    /// # Panics
    ///
    /// Panics if a parameter with the same name already exists.
    #[must_use]
    pub fn add(&self, name: &str, default: f64) -> ParamRef {
        let mut entries = write(&self.0.inner);
        assert!(
            !entries.names.iter().any(|n| n == name),
            "parameter '{name}' already exists"
        );
        let id = entries.names.len();
        entries.names.push(name.to_string());
        entries.values.push(default);
        drop(entries);
        ParamRef {
            id,
            store: Arc::clone(&self.0),
        }
    }

    /// Set a parameter value by name.
    ///
    /// # Panics
    ///
    /// Panics if the parameter name is not found.
    pub fn set(&self, name: &str, value: f64) {
        let mut entries = write(&self.0.inner);
        let idx = entries.names.iter().position(|n| n == name);
        assert!(idx.is_some(), "parameter '{name}' not found");
        if let Some(i) = idx {
            entries.values[i] = value;
        }
    }

    /// Get a parameter value by name. Returns `None` if not found.
    #[must_use]
    pub fn get(&self, name: &str) -> Option<f64> {
        let entries = read(&self.0.inner);
        entries
            .names
            .iter()
            .position(|n| n == name)
            .map(|i| entries.values[i])
    }

    /// List all parameter names.
    #[must_use]
    pub fn names(&self) -> Vec<String> {
        let entries = read(&self.0.inner);
        entries.names.clone()
    }
}

impl Default for ParamStore {
    fn default() -> Self {
        Self::new()
    }
}

impl ParamStoreData {
    /// Read a parameter value by index (called during evaluation).
    pub(crate) fn get_by_id(&self, id: usize) -> f64 {
        let entries = read(&self.inner);
        entries.values[id]
    }

    /// Find a parameter index by name.
    pub(crate) fn find_name(&self, name: &str) -> Option<usize> {
        let entries = read(&self.inner);
        entries.names.iter().position(|n| n == name)
    }

    /// Get the name of a parameter by index.
    pub(crate) fn name_of(&self, id: usize) -> String {
        let entries = read(&self.inner);
        entries.names[id].clone()
    }

    /// Set a parameter value by index.
    pub(crate) fn set_by_id(&self, id: usize, value: f64) {
        let mut entries = write(&self.inner);
        entries.values[id] = value;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn basic_add_get_set() {
        let store = ParamStore::new();
        let r = store.add("radius", 5.0);
        assert!((r.value() - 5.0).abs() < f64::EPSILON);
        assert!((store.get("radius").unwrap_or(0.0) - 5.0).abs() < f64::EPSILON);

        store.set("radius", 10.0);
        assert!((r.value() - 10.0).abs() < f64::EPSILON);
        assert!((store.get("radius").unwrap_or(0.0) - 10.0).abs() < f64::EPSILON);
    }

    #[test]
    fn multiple_params() {
        let store = ParamStore::new();
        let _r = store.add("radius", 5.0);
        let _k = store.add("blend_k", 1.0);
        assert_eq!(store.names().len(), 2);
        assert!(store.get("radius").is_some());
        assert!(store.get("blend_k").is_some());
        assert!(store.get("nonexistent").is_none());
    }

    #[test]
    #[should_panic(expected = "already exists")]
    fn duplicate_name_panics() {
        let store = ParamStore::new();
        let _r1 = store.add("radius", 5.0);
        let _r2 = store.add("radius", 10.0);
    }

    #[test]
    #[should_panic(expected = "not found")]
    fn set_nonexistent_panics() {
        let store = ParamStore::new();
        store.set("nonexistent", 1.0);
    }

    #[test]
    fn clone_shares_store() {
        let store = ParamStore::new();
        let _r = store.add("radius", 5.0);
        let store2 = store.clone();
        store2.set("radius", 10.0);
        assert!((store.get("radius").unwrap_or(0.0) - 10.0).abs() < f64::EPSILON);
    }
}
