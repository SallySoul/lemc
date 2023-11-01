use crate::nalgebra_types::*;
use crate::aabb::AABB;
use crate::trimesh::ply::*;
use crate::trimesh::stl::*;
use anyhow::*;

pub type IndexTriplet = nalgebra::Vector3<i32>;

/// The result for doing a Line intersection check.
#[derive(PartialEq, Debug)]
pub enum LineIntersectionResult<T: FloatField> {
    /// There is no intersection
    None,

    /// The intersection occurs at the t parameter
    Some(T),
}

/// Triangle Mesh class.
/// Tracks triangle -> node topology.
/// Can provide surface quadrature.
#[derive(Default)]
pub struct TriangleMesh<T: FloatField> {
    pub node_positions: Vec<SVector<T, 3>>,
    pub triangle_indices: Vec<[usize; 3]>,
}

impl<T: FloatField> TriangleMesh<T> {
    /// TODO
    pub fn new() -> TriangleMesh<T> {
        TriangleMesh {
            node_positions: Vec::new(),
            triangle_indices: Vec::new(),
        }
    }

    /// Read a TriangleMesh from file.
    /// `.ply` and `.stl` files are supported.
    pub fn from_file<P: AsRef<std::path::Path>>(p: &P) -> anyhow::Result<TriangleMesh<T>> {
        let path: &std::path::Path = p.as_ref();
        let maybe_extension = path.extension();

        let extension = match maybe_extension {
            Some(ext) => ext.to_str().with_context(|| {
                format!(
                    "Failed to convert file extension to str: {}",
                    path.display()
                )
            })?,
            None => {
                return Err(anyhow!(
                    "No extension on TriangleMesh file: {}",
                    path.display()
                ));
            }
        };

        match extension {
            "stl" => read_stl_file(p)
                .with_context(|| format!("Failed to read stl file: {}", path.display())),
            "ply" => read_ply_file(p)
                .with_context(|| format!("Failed to read ply file: {}", path.display())),
            ext => Err(anyhow!(format!(
                "Unknown file type for TriangleMesh: {}",
                ext
            ))),
        }
    }

    /// TODO
    pub fn with_capacity(node_len: usize, triangle_len: usize) -> TriangleMesh<T> {
        TriangleMesh {
            node_positions: Vec::with_capacity(node_len),
            triangle_indices: Vec::with_capacity(triangle_len),
        }
    }

    /// TODO
    pub fn node_len(&self) -> usize {
        self.node_positions.len()
    }

    /// TODO
    pub fn triangle_len(&self) -> usize {
        self.triangle_indices.len()
    }

    /// TODO
    pub fn add_node(&mut self, position: SVector<T, 3>) -> usize {
        let result = self.node_len();
        self.node_positions.push(position);
        result
    }

    /// TODO
    pub fn node(&self, index: usize) -> &SVector<T, 3> {
        &self.node_positions[index]
    }

    /// TODO
    pub fn add_triangle(&mut self, indices: [usize; 3]) -> usize {
        let result = self.triangle_len();
        self.triangle_indices.push(indices);
        result
    }

    /// TODO
    pub fn triangle(&self, index: usize) -> &[usize; 3] {
        &self.triangle_indices[index]
    }

    /// TODO
    pub fn triangle_normal(&self, index: usize) -> SVector<T, 3>{
        let &[n0, n1, n2] = self.triangle(index);
        let a = self.node(n1) - self.node(n0);
        let b = self.node(n2) - self.node(n0);
        a.cross(&b).normalize()
    }

    /// TODO
    pub fn clear(&mut self) {
        self.node_positions.clear();
        self.triangle_indices.clear();
    }

    /// TODO
    pub fn triangle_area(&self, t: usize) -> T {
        let [n0, n1, n2] = self.triangle(t);
        let a = self.node(*n1) - self.node(*n0);
        let b = self.node(*n2) - self.node(*n0);
        (a.cross(&b) * T::half()).norm()
    }

    /// TODO
    pub fn surface_area(&self) -> T {
        let mut surface_area = T::zero();
        for t in 0..self.triangle_len() {
            surface_area += self.triangle_area(t);
        }
        surface_area
    }

    /// TODO
    pub fn bounds(&self) -> AABB<T, 3> {
        let mut aabb = AABB::blank();
        for n in &self.node_positions {
            aabb.mut_add_point(n);
        }
        aabb
    }

    /// TODO
    pub fn line_intersects_triangle(&self, t: usize, a: &SVector<T, 3>, b: &SVector<T, 3>) -> LineIntersectionResult<T> {
        let [n0, n1, n2] = self.triangle(t);

        // Find intersection with triangle plane
        // Duplicate code from plane intersection so we can catch degenerate case
        let normal = self.triangle_normal(t);
        let a_distance_from_plane = normal.dot(&(self.node(*n0) - a));
        let len_perpendicular_to_plane = normal.dot(&(b - a));

        // Degenerate case where triangle is parallel to the ray
        if len_perpendicular_to_plane == T::zero() {
            return LineIntersectionResult::None;
        }
        let t = a_distance_from_plane / len_perpendicular_to_plane;
        if !(T::zero()..=T::one()).contains(&t) {
            return LineIntersectionResult::None;
        }
        let intersection_point: SVector<T, 3> = a + (b - a) * t;

        // Now find barycentric coordinates of intersection intersection_point
        // from: http://www.blackpawn.com/texts/pointinpoly/default.html
        let v0 = self.node(*n2) - self.node(*n0);
        let v1 = self.node(*n1) - self.node(*n0);
        let v2 = intersection_point - self.node(*n0);
        let dot00 = v0.dot(&v0);
        let dot01 = v0.dot(&v1);
        let dot02 = v0.dot(&v2);
        let dot11 = v1.dot(&v1);
        let dot12 = v1.dot(&v2);
        let invdenom = T::one() / (dot00 * dot11 - dot01 * dot01);
        let u = (dot11 * dot02 - dot01 * dot12) * invdenom;
        let v = (dot00 * dot12 - dot01 * dot02) * invdenom;
        if (u >= T::zero()) && (v >= T::zero()) && (u + v <= T::one()) {
            return LineIntersectionResult::Some(t);
        }
        LineIntersectionResult::None
    }

    /// TODO
    pub fn aabb_intersects_triangle(&self, t: usize, aabb: &AABB<T, 3>) -> bool {
        // This is an implementation of the method described in
        // "Fast 3D Triangle-Box Overlap Testing"
        // by Tomas Akenine-Moller
        //
        // The basic notion is based on the seperating axis theorem
        //  a) Two convex polyhedra are seperate if there exists a "seperating axis"
        //     such that the projections of the polyhedra onto the axis do not overlap
        //  b) Two polyhedra are seperable iff a seperating axis can be formed by a
        //     normal to one the polyhedra's face, or the cross product of a pair of
        //     edges from both.
        //
        // This gives us 13 possible seperating axis to test for a AABB and triangle
        //  - x, y, and z axis
        //  - the normal to the triangle
        //  - The nine unique axis formed from cube edges cross triangle edges,
        //
        // So we construct all the possible seperating axis and do the projection test
        // for each.

        // Center the problem so that the origin is the box center
        let origin = aabb.centroid();
        let [n0, n1, n2] = self.triangle(t);
        let p0 = self.node(*n0) - origin;
        let p1 = self.node(*n1) - origin;
        let p2 = self.node(*n2) - origin;
        let edge_0 = self.node(*n1) - self.node(*n0);
        let edge_1 = self.node(*n2) - self.node(*n1);
        let edge_2 = self.node(*n0) - self.node(*n2);
        let half_diagonal = aabb.diagonal() * T::half();
        let is_seperating_axis = |axis: SVector<T, 3>| -> bool {
            // If the edges are not linearly independent, then this is not a valid
            // axis. This case was not mentioned in the paper. However, if this is the
            // case then the triangle has at least one edge parallel to the x, y, or z
            // axis, so we are testing the other two orthogonal axis anyway
            if axis.norm_squared() <= T::min_value().unwrap() {
                return false;
            }

            // Otherwise, project box and triangle onto axis
            let box_projection_radius = half_diagonal.dot(&axis.abs());
            let node_projections = vector![p0.dot(&axis), p1.dot(&axis), p2.dot(&axis)];
            let node_projection_min = node_projections.min();
            let node_projection_max = node_projections.max();
            if node_projection_min >= box_projection_radius
                || node_projection_max <= -box_projection_radius
            {
                return true;
            }
            false
        };

        let has_separating_axis = // Force these test to be on their own lines
            is_seperating_axis(SVector::<T, 3>::x())
            || is_seperating_axis(SVector::<T, 3>::y())
            || is_seperating_axis(SVector::<T, 3>::z())
            || is_seperating_axis(self.triangle_normal(t))
            || is_seperating_axis(edge_0.cross(&SVector::<T, 3>::x()))
            || is_seperating_axis(edge_1.cross(&SVector::<T, 3>::x()))
            || is_seperating_axis(edge_2.cross(&SVector::<T, 3>::x()))
            || is_seperating_axis(edge_0.cross(&SVector::<T, 3>::y()))
            || is_seperating_axis(edge_1.cross(&SVector::<T, 3>::y()))
            || is_seperating_axis(edge_2.cross(&SVector::<T, 3>::y()))
            || is_seperating_axis(edge_0.cross(&SVector::<T, 3>::z()))
            || is_seperating_axis(edge_1.cross(&SVector::<T, 3>::z()))
            || is_seperating_axis(edge_2.cross(&SVector::<T, 3>::z()));

        !has_separating_axis
    }

    /// TODO
    pub fn triangle_aabb(&self, t: usize) -> AABB<T, 3> {
        let [n0, n1, n2] = self.triangle(t);
        let mut result = AABB::from_point(self.node(*n0).clone());
        result.mut_add_point(self.node(*n1));
        result.mut_add_point(self.node(*n2));
        result
    }
}

#[cfg(test)]
mod unit_test {
    use super::*;
    use crate::geometric_types::vec3;

    #[test]
    fn empty_mesh() {
        let m = TriangleMesh::new();
        assert_eq!(m.node_len(), 0);
        assert_eq!(m.triangle_len(), 0);
        assert_eq!(m.node_positions.capacity(), 0);
        assert_eq!(m.triangle_indices.capacity(), 0);
    }

    #[test]
    fn add_node_and_node() {
        let mut m = TriangleMesh::new();
        let v1 = m.add_node(vector![0.0, 0.0, 0.0]);
        assert_eq!(m.node_len(), 1);
        assert_eq!(v1, 0);

        let v2 = m.add_node(vector![1.0, 1.0, 1.0]);
        assert_eq!(m.node_len(), 2);
        assert_eq!(v2, 1);
        assert_eq!(
            nalgebra::partial_cmp(m.node(0), &vector![0.0, 0.0, 0.0]),
            Some(std::cmp::Ordering::Equal)
        );
        assert_eq!(
            nalgebra::partial_cmp(m.node(1), &vector![1.0, 1.0, 1.0]),
            Some(std::cmp::Ordering::Equal)
        );
    }

    #[test]
    fn add_triangle_and_triangle_and_len_and_clear() {
        let mut m = TriangleMesh::new();
        for i in 0..9 {
            assert_eq!(m.add_node(vector![i as f64, i as f64, i as f64]), i);
        }
        assert_eq!(m.node_len(), 9);
        assert_eq!(m.add_triangle([0, 1, 2]), 0);
        assert_eq!(m.add_triangle([2, 3, 4]), 1);
        assert_eq!(m.add_triangle([5, 6, 7]), 2);
        assert_eq!(m.triangle_len(), 3);
        assert_eq!(*m.triangle(0), [0, 1, 2]);
        assert_eq!(*m.triangle(1), [2, 3, 4]);
        assert_eq!(*m.triangle(2), [5, 6, 7]);

        m.clear();
        assert_eq!(m.node_len(), 0);
        assert_eq!(m.triangle_len(), 0);
    }

    #[test]
    fn with_capacity() {
        let m = TriangleMesh::with_capacity(5, 7);
        assert_eq!(m.node_len(), 0);
        assert_eq!(m.triangle_len(), 0);
        assert_eq!(m.node_positions.capacity(), 5);
        assert_eq!(m.triangle_indices.capacity(), 7);
    }

    #[test]
    fn triangle_normal() {
        let mut m = super::TriangleMesh::new();
        let indices = [
            m.add_node(vector![0.0, 0.0, 0.0]),
            m.add_node(vector![1.0, 0.0, 0.0]),
            m.add_node(vector![0.0, 1.0, 0.0]),
        ];
        m.add_triangle(indices);

        assert_eq!(m.triangle_normal(0), vector![0.0, 0.0, 1.0]);
    }

    #[test]
    fn triangle_area() {
        let mut m = super::TriangleMesh::new();
        let indices = [
            m.add_node(vector![0.0, 0.0, 0.0]),
            m.add_node(vector![1.0, 0.0, 0.0]),
            m.add_node(vector![0.0, 1.0, 0.0]),
        ];
        m.add_triangle(indices);

        assert_eq!(m.triangle_area(0), 0.5);
    }

    #[test]
    fn surface_area() {
        let mut m = super::TriangleMesh::new();
        let indices = [
            m.add_node(vector![0.0, 0.0, 0.0]),
            m.add_node(vector![1.0, 0.0, 0.0]),
            m.add_node(vector![0.0, 1.0, 0.0]),
        ];
        m.add_triangle(indices);
        assert_eq!(m.surface_area(), 0.5);

        let indices = [
            m.add_node(vector![0.0, 0.0, 0.0]),
            m.add_node(vector![-1.0, 0.0, 0.0]),
            m.add_node(vector![0.0, -1.0, 0.0]),
        ];
        m.add_triangle(indices);
        assert_eq!(m.surface_area(), 1.0);
    }

    #[test]
    fn bounds() {
        let mut m = super::TriangleMesh::new();
        m.add_node(vector![0.0, 0.0, 0.0]);
        assert_eq!(
            m.bounds(),
            AABB::from_points(&vector![0.0, 0.0, 0.0], &vector![0.0, 0.0, 0.0])
        );

        m.add_node(vector![1.0, 1.0, 1.0]);
        assert_eq!(
            m.bounds(),
            AABB::from_points(&vector![0.0, 0.0, 0.0], &vector![1.0, 1.0, 1.0])
        );

        m.add_node(vector![-1.0, -1.0, 2.0]);
        assert_eq!(
            m.bounds(),
            AABB::from_points(&vector![-1.0, -1.0, 0.0], &vector![1.0, 1.0, 2.0])
        );
    }

    #[test]
    fn ray_intersection() {
        let mut m = super::TriangleMesh::new();
        let indices = [
            m.add_node(vector![0.0, 0.0, 0.0]),
            m.add_node(vector![1.0, 0.0, 0.0]),
            m.add_node(vector![0.0, 1.0, 0.0]),
        ];
        m.add_triangle(indices);

        assert_eq!(
            LineIntersectionResult::Some(0.0),
            m.line_intersects_triangle(0, &vector![0.4, 0.4, 0.0], &vector![0.4, 0.4, 1.0])
        );
        assert_eq!(
            LineIntersectionResult::Some(0.0),
            m.line_intersects_triangle(0, &vector![0.0, 0.0, 0.0], &vector![0.4, 0.4, 1.0])
        );
        assert_eq!(
            LineIntersectionResult::None,
            m.line_intersects_triangle(0, &vector![1.0, 0.0, 0.0], &vector![1.0, 1.0, 0.0])
        );
        assert_eq!(
            LineIntersectionResult::None,
            m.line_intersects_triangle(0, &vector![-1.0, -1.0, 0.0], &vector![-1.0, -1.0, -1.0])
        );
    }

    #[test]
    fn is_aabb_intersection_0() {
        let mut m = super::TriangleMesh::new();
        let indices = [
            m.add_node(vector![0.0, 0.0, 0.1]),
            m.add_node(vector![1.0, 0.0, 0.1]),
            m.add_node(vector![0.0, 1.0, 0.1]),
        ];
        m.add_triangle(indices);
        let b = AABB::from_points(vector![0.0, 0.0, 0.0], vector![1.0, 1.0, 1.0]);
        assert!(m.aabb_intersects_triangle(0, &b));
    }

    #[test]
    fn not_aabb_intersection_1() {
        let mut m = super::TriangleMesh::new();
        let indices = [
            m.add_node(vector![-0.1, 0.0, 0.0]),
            m.add_node(vector![-1.0, 1.0, 0.0]),
            m.add_node(vector![-2.0, 0.0, 1.0]),
        ];
        m.add_triangle(indices);
        let b = AABB::from_points(vector![0.0, 0.0, 0.0], vector![1.0, 1.0, 1.0]);
        assert!(!m.aabb_intersects_triangle(0, &b));
    }

    #[test]
    fn not_aabb_intersection_2() {
        let mut m = super::TriangleMesh::new();
        let indices = [
            m.add_node(vector![0.0, -0.1, 0.0]),
            m.add_node(vector![1.0, -1.0, 0.0]),
            m.add_node(vector![0.0, -2.0, 1.0]),
        ];
        m.add_triangle(indices);
        let b = AABB::from_points(&vector![0.0, 0.0, 0.0], &vector![1.0, 1.0, 1.0]);
        assert!(!m.aabb_intersects_triangle(0, &b));
    }

    #[test]
    fn not_aabb_intersection_3() {
        let mut m = super::TriangleMesh::new();
        let indices = [
            m.add_node(vector![0.0, 0.0, -0.1]),
            m.add_node(vector![1.0, 0.0, -1.0]),
            m.add_node(vector![0.0, 1.0, -2.0]),
        ];
        m.add_triangle(indices);
        let b = AABB::from_points(&vector![0.0, 0.0, 0.0], &vector![1.0, 1.0, 1.0]);
        assert!(!m.aabb_intersects_triangle(0, &b));
    }

    #[test]
    fn not_aabb_intersection_4() {
        let mut m = super::TriangleMesh::new();
        let indices = [
            m.add_node(vector![1.0, -1.2, 0.5]),
            m.add_node(vector![-1.0, 1.0, 0.0]),
            m.add_node(vector![-1.0, 1.0, 1.0]),
        ];
        m.add_triangle(indices);
        let b = AABB::from_points(&vector![0.0, 0.0, 0.0], &vector![1.0, 1.0, 1.0]);
        assert!(!m.aabb_intersects_triangle(0, &b));
    }

    #[test]
    fn not_aabb_intersection_rest() {
        let b = AABB::from_points(&vector![-0.5, -0.5, -0.5], &vector![0.5, 0.5, 0.5]);
        let points = [
            vector![-0.2, 0.0, 1.2],
            vector![-0.2, 1.2, 0.0],
            vector![0.2, 2.0, 2.0],
        ];

        for bounding_box_axis in 0..3 {
            for edge in 0..3 {
                let mut m = super::TriangleMesh::new();
                let indices = [
                    m.add_node(SVector::new(
                        points[edge % 3][bounding_box_axis % 3],
                        points[edge % 3][(bounding_box_axis + 1) % 3],
                        points[edge % 3][(bounding_box_axis + 2) % 3],
                    )),
                    m.add_node(SVector::new(
                        points[(edge + 1) % 3][bounding_box_axis % 3],
                        points[(edge + 1) % 3][(bounding_box_axis + 1) % 3],
                        points[(edge + 1) % 3][(bounding_box_axis + 2) % 3],
                    )),
                    m.add_node(SVector::new(
                        points[(edge + 2) % 3][bounding_box_axis % 3],
                        points[(edge + 2) % 3][(bounding_box_axis + 1) % 3],
                        points[(edge + 2) % 3][(bounding_box_axis + 2) % 3],
                    )),
                ];
                m.add_triangle(indices);
                assert!(!m.aabb_intersects_triangle(0, &b));
            }
        }
    }

    #[test]
    fn triangle_aabb() {
        let mut m = super::TriangleMesh::new();
        let indices = [
            m.add_node(vector![0.0, 0.0, 3.0]),
            m.add_node(vector![0.0, 5.0, 0.0]),
            m.add_node(vector![7.0, 0.0, 0.0]),
        ];
        m.add_triangle(indices);
        let b = m.triangle_aabb(0);
        assert_eq!(b.min, vector![0.0, 0.0, 0.0]);
        assert_eq!(b.max, vector![7.0, 5.0, 3.0]);
    }

    #[test]
    fn debug_intersection_result() {
        let s0 = format!("{:?}", LineIntersectionResult::None);
        assert_eq!(s0, "None");

        let s1 = format!("{:?}", LineIntersectionResult::Some(1.0));
        assert_eq!(s1, "Some(1.0)");
    }
}
