use crate::aabb::*;
use crate::nalgebra_types::*;

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum NodeType {
    Parent,
    Leaf,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct NodeId {
    t: NodeType,
    i: usize,
}

impl NodeId {
    pub fn new(t: NodeType, i: usize) -> NodeId {
        NodeId { t, i }
    }

    pub fn root() -> NodeId {
        NodeId {
            t: NodeType::Parent,
            i: 0,
        }
    }
}

pub struct Object<T: RealField, const D: usize> {
    pub b: AABB<T, D>,
    pub id: usize,
}

pub struct ParentNode<T: RealField, const D: usize> {
    pub b: AABB<T, D>,
    pub children: Vec<NodeId>,
}

pub struct LeafNode<T: RealField, const D: usize> {
    pub b: AABB<T, D>,
    pub objects: Vec<usize>,
}

pub struct NodeStore<T: RealField, const D: usize> {
    pub parents: Vec<ParentNode<T, D>>,
    pub leaves: Vec<LeafNode<T, D>>,
    pub objects: Vec<Object<T, D>>,
}

impl<T: RealField, const D: usize> NodeStore<T, D> {
    pub fn empty() -> NodeStore<T, D> {
        NodeStore {
            parents: Vec::new(),
            leaves: Vec::new(),
            objects: Vec::new(),
        }
    }

    pub fn insert_object(&mut self, object: Object<T, D>) -> usize {
        let id = self.objects.len();
        self.objects.push(object);
        id
    }

    pub fn insert_parent(&mut self) -> NodeId {
        let id = self.parents.len();
        self.parents.push(ParentNode {
            children: Vec::new(),
            b: AABB::blank(),
        });
        NodeId::new(NodeType::Parent, id)
    }

    pub fn insert_leaf(&mut self, node: LeafNode<T, D>) -> NodeId {
        let id = self.leaves.len();
        self.leaves.push(node);
        NodeId::new(NodeType::Leaf, id)
    }

    pub fn get_node_bounds(&self, id: NodeId) -> &AABB<T, D> {
        match id.t {
            NodeType::Parent => &self.parents[id.i].b,
            NodeType::Leaf => &self.leaves[id.i].b,
        }
    }
}
