use crate::nalgebra_types::*;
use crate::trimesh::TriangleMesh;
use ply_rs::parser;
use ply_rs::ply;
use std::io::BufReader;

struct VertexWrapper<T: FloatField>(SVector<T, 3>);

struct FaceWrapper([usize; 3]);

impl<T: FloatField> ply::PropertyAccess for VertexWrapper<T> {
    fn new() -> Self {
        VertexWrapper(SVector::zeros())
    }

    fn set_property(&mut self, key: String, property: ply::Property) {
        match (key.as_ref(), property) {
            ("x", ply::Property::Float(v)) => self.0.x = T::from_f32(v).unwrap(),
            ("y", ply::Property::Float(v)) => self.0.y = T::from_f32(v).unwrap(),
            ("z", ply::Property::Float(v)) => self.0.z = T::from_f32(v).unwrap(),
            (k, _) => panic!("Vertex: Unexpected key/value combination: key: {}", k),
        }
    }
}

// same thing for Face
impl ply::PropertyAccess for FaceWrapper {
    fn new() -> Self {
        FaceWrapper([0; 3])
    }
    fn set_property(&mut self, key: String, property: ply::Property) {
        match (key.as_ref(), property) {
            ("vertex_indices", ply::Property::ListInt(vec)) => {
                assert_eq!(vec.len(), 3);
                self.0[0] = vec[0] as usize;
                self.0[1] = vec[1] as usize;
                self.0[2] = vec[2] as usize;
            }
            (k, _) => panic!("Face: Unexpected key/value combination: key: {}", k),
        }
    }
}

/// Read a TriangleMesh in from a ply file
pub fn read_ply_file<P: AsRef<std::path::Path>, T: FloatField>(path: &P) -> Result<TriangleMesh<T>, anyhow::Error> {
    let input_file = std::fs::File::open(path)?;
    let mut input_reader = BufReader::new(input_file);

    let vertex_parser = parser::Parser::<VertexWrapper<T>>::new();
    let face_parser = parser::Parser::<FaceWrapper>::new();
    let header = vertex_parser.read_header(&mut input_reader)?;
    let mut vertex_list = Vec::new();
    let mut face_list = Vec::new();
    for (_ignore_key, element) in &header.elements {
        // we could also just parse them in sequence, but the file format might change
        match element.name.as_ref() {
            "vertex" => {
                vertex_list =
                    vertex_parser.read_payload_for_element(&mut input_reader, element, &header)?;
            }
            "face" => {
                face_list =
                    face_parser.read_payload_for_element(&mut input_reader, element, &header)?;
            }
            _ => panic!("Enexpeced element!"),
        }
    }
    let node_positions = unsafe { std::mem::transmute(vertex_list) };
    let triangle_indices = unsafe { std::mem::transmute(face_list) };
    Ok(TriangleMesh {
        node_positions,
        triangle_indices,
    })
}

#[cfg(test)]
mod unit_tests {
    use super::*;

    #[test]
    fn read_ply() {
        let mut box_path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"));
        box_path.push("test_assets/block.ply");

        let result = read_ply_file(&box_path);

        match result {
            Err(error) => {
                println!("Path: {}", box_path.display());
                println!("Error: {}", error);
                panic!();
            }
            Ok(mesh) => {
                assert_eq!(mesh.node_len(), 8);
                assert_eq!(mesh.triangle_len(), 12);
            }
        }
    }
}
