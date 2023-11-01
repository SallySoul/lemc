use crate::bvh_node_store::*;
use crate::nalgebra_types::*;

pub struct InsertBuilder<T: RealField, const D: usize> {
    store: NodeStore<T, D>,
    root: NodeId,
    object_id_buffer: Vec<usize>,
    objects_per_leaf: usize,
}

impl<T: RealField, const D: usize> InsertBuilder<T, D> {
    pub fn empty() -> InsertBuilder<T, D> {
        let mut store = NodeStore::empty();
        let root = store.insert_parent();
        InsertBuilder {
            store,
            root,
            object_id_buffer: Vec::new(),
            objects_per_leaf: 3,
        }
    }
}
