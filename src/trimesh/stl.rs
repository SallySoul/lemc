// This file is originally from stl_io. It has been hacked up significantly.
// https://github.com/hmeyer/stl_io/blob/master/src/lib.rs
use crate::nalgebra_types::*;
use crate::trimesh::TriangleMesh;
use byteorder::{LittleEndian, ReadBytesExt, WriteBytesExt};
use std::io::{BufRead, BufReader, BufWriter};
use std::io::{Read, Result, Write};
use std::iter::Iterator;

pub fn write_stl<W, T>(writer: &mut W, mesh: &TriangleMesh<T>) -> Result<()>
where
    W: ::std::io::Write,
    T: FloatField,
{
    let mut writer = BufWriter::new(writer);

    // Write 80 byte header
    writer.write_all(&[0u8; 80])?;
    writer.write_u32::<LittleEndian>(mesh.triangle_len() as u32)?;
    for t in 0..mesh.triangle_len() {
        for f in &mesh.triangle_normal(t) {
            writer.write_f32::<LittleEndian>((*f).as_f32())?;
        }
        for &n in mesh.triangle(t) {
            for c in mesh.node(n) {
                writer.write_f32::<LittleEndian>((*c).as_f32())?;
            }
        }
        // Attribute byte count
        writer.write_u16::<LittleEndian>(0)?;
    }
    writer.flush()
}

pub fn read_stl<R, T>(read: &mut R) -> Result<TriangleMesh<T>>
where
    R: ::std::io::Read + ::std::io::Seek,
    T: FloatField,
{
    if is_ascii_stl(read).is_ok() {
        read_ascii_stl(read)
    } else {
        read_binary_stl(read)
    }
}

fn read_binary_stl<R: std::io::Read + std::io::Seek, T: FloatField>(
    read: &mut R,
) -> Result<TriangleMesh<T>> {
    let mut reader = BufReader::new(read);
    reader.read_exact(&mut [0u8; 80])?;
    let num_triangles = reader.read_u32::<LittleEndian>()? as usize;
    let mut result = TriangleMesh::<T>::with_capacity(num_triangles * 3, num_triangles);

    for _ in 0..num_triangles {
        let mut _normal: SVector<T, 3> = SVector::zero();
        for f in &mut _normal {
            *f = T::frm_f32(reader.read_f32::<LittleEndian>()?);
        }

        let mut indices = [0; 3];
        for i in &mut indices {
            let mut position: SVector<T, 3> = SVector::zero();
            for c in &mut position {
                *c = T::frm_f32(reader.read_f32::<LittleEndian>()?);
            }
            *i = result.add_node(position);
        }
        reader.read_u16::<LittleEndian>()?;
        result.add_triangle(indices);
    }

    Ok(result)
}

pub fn is_ascii_stl<R: ::std::io::Read + ::std::io::Seek>(read: &mut R) -> Result<()> {
    let mut header = String::new();
    let maybe_read_error = BufReader::new(&mut *read).read_line(&mut header);
    // Try to seek back to start before evaluating potential read errors.
    read.seek(::std::io::SeekFrom::Start(0))?;
    maybe_read_error?;
    if header.starts_with("solid ") {
        Ok(())
    } else {
        Err(::std::io::Error::new(
            ::std::io::ErrorKind::InvalidData,
            format!("ascii starts with solid, found: {:?}", header),
        ))
    }
}

fn ascii_expect_static<L>(lines: &mut L, expectation: &[&str]) -> Result<()>
where
    L: std::iter::Iterator<Item = Result<Vec<String>>>,
{
    if let Some(line) = lines.next() {
        let line = line?;
        if line != expectation {
            return Err(::std::io::Error::new(
                ::std::io::ErrorKind::InvalidData,
                format!("expected {:?}, got {:?}", expectation, line),
            ));
        }
    } else {
        return Err(::std::io::Error::new(
            ::std::io::ErrorKind::UnexpectedEof,
            format!("EOF while expecting {:?}", expectation),
        ));
    }
    Ok(())
}

fn ascii_tokens_to_vec3<T: FloatField>(tokens: &[String]) -> Result<SVector<T, 3>> {
    let mut result = SVector::zeros();
    for i in 0..3 {
        // This is some bullshit:
        // https://github.com/rust-num/num-traits/issues/260 
        result[i] = if let Ok(r) = T::from_str_radix(&tokens[i], 10) {
            r
        } else {
            panic!("Failed to from_str_radix");
        };
        if !result[i].is_finite() {
            return Err(::std::io::Error::new(
                ::std::io::ErrorKind::InvalidData,
                format!("expected finite f64, got {} which is", result[i],),
            ));
        }
    }
    Ok(result)
}

fn read_ascii_stl<R: std::io::Read, T: FloatField>(read: &mut R) -> Result<TriangleMesh<T>> {
    // Only call if is_ascii_stl passes
    let mut lines = BufReader::new(read).lines();
    lines.next();

    let mut tokens = lines.map(|result| {
        result.map(|l| {
            // Make lines into iterator over vectors of tokens
            l.split_whitespace()
                .map(|t| t.to_string())
                .collect::<Vec<_>>()
        })
    });

    let mut result: TriangleMesh<T> = TriangleMesh::<T>::new();
    loop {
        let face_header: Option<Result<Vec<String>>> = tokens.next();
        if face_header.is_none() {
            return Err(::std::io::Error::new(
                ::std::io::ErrorKind::UnexpectedEof,
                "EOF while expecting facet or endsolid.",
            ));
        }
        let face_header = face_header.unwrap()?;
        if !face_header.is_empty() && face_header[0] == "endsolid" {
            break;
        }
        if face_header.len() != 5 || face_header[0] != "facet" || face_header[1] != "normal" {
            return Err(::std::io::Error::new(
                ::std::io::ErrorKind::InvalidData,
                format!("invalid facet header: {:?}", face_header),
            ));
        }
        let _normal = ascii_tokens_to_vec3::<T>(&face_header[2..5])?;
        ascii_expect_static(&mut tokens, &["outer", "loop"])?;
        let mut triangle_indices = [0; 3];
        for vertex_result in &mut triangle_indices {
            if let Some(line) = tokens.next() {
                let line = line?;
                if line.len() != 4 || line[0] != "vertex" {
                    return Err(::std::io::Error::new(
                        ::std::io::ErrorKind::InvalidData,
                        format!("vertex f32 f32 f32, got {:?}", line),
                    ));
                }
                *vertex_result = result.add_node(ascii_tokens_to_vec3(&line[1..4])?);
            } else {
                return Err(::std::io::Error::new(
                    ::std::io::ErrorKind::UnexpectedEof,
                    "EOF while expecting vertex",
                ));
            }
        }
        result.add_triangle(triangle_indices);
        ascii_expect_static(&mut tokens, &["endloop"])?;
        ascii_expect_static(&mut tokens, &["endfacet"])?;
    }
    Ok(result)
}

pub fn write_stl_file<P: AsRef<std::path::Path>, T: FloatField>(path: &P, mesh: &TriangleMesh<T>) -> Result<()> {
    let output_file = std::fs::File::create(path)?;
    let mut output_writer = BufWriter::new(output_file);
    write_stl(&mut output_writer, mesh)?;
    Ok(())
}

pub fn read_stl_file<P: AsRef<std::path::Path>, T: FloatField>(path: &P) -> Result<TriangleMesh<T>> {
    let input_file = std::fs::File::open(path)?;
    let mut input_reader = BufReader::new(input_file);
    read_stl(&mut input_reader)
}

#[cfg(test)]
mod unit_tests {
    use super::*;
    use approx::relative_eq;

    const BUNNY_99: &[u8] = include_bytes!("../../test_assets/bunny_99.stl");
    const BUNNY_99_ASCII: &[u8] = include_bytes!("../../test_assets/bunny_99_ascii.stl");

    #[test]
    fn ascii_expect_static() {
        {
            let mut tokens = vec![Ok(vec![String::from("a"), String::from("b")])];
            let mut i = tokens.drain(0..);
            let result0 = super::ascii_expect_static(&mut i, &["a", "b"]);
            assert!(result0.is_ok());
        }

        {
            let mut tokens = vec![Ok(vec![String::from("a"), String::from("b")])];
            let mut i = tokens.drain(0..);
            let result0 = super::ascii_expect_static(&mut i, &["b", "a"]);
            assert!(result0.is_err());
            assert_eq!(
                result0.as_ref().err().unwrap().kind(),
                ::std::io::ErrorKind::InvalidData
            );
        }

        {
            let mut tokens = vec![];
            let mut i = tokens.drain(0..);
            let result0 = super::ascii_expect_static(&mut i, &["b", "a"]);
            assert!(result0.is_err());
            assert_eq!(
                result0.as_ref().err().unwrap().kind(),
                ::std::io::ErrorKind::UnexpectedEof
            );
        }
    }

    #[test]
    fn read_ascii_stl_simple_success() {
        let mut reader = ::std::io::Cursor::new(
            b"solid foobar
        facet normal 1.1 0.2 0.3
            outer loop
                vertex 1 2 3
                vertex 4 5 6e-15
                vertex 7 8 9.87654321
            endloop
        endfacet
        endsolid foobar"
                .to_vec(),
        );
        let maybe_mesh = read_stl(&mut reader);
        assert!(maybe_mesh.is_ok());
        let mesh = maybe_mesh.unwrap();
        assert_eq!(mesh.node_len(), 3);
        assert_eq!(*mesh.node(0), vec3![1.0, 2.0, 3.0]);
        assert_eq!(*mesh.node(1), vec3![4.0, 5.0, 6e-15]);
        assert_eq!(*mesh.node(2), vec3![7.0, 8.0, 9.87654321]);
        assert_eq!(mesh.triangle_len(), 1);
        assert_eq!(*mesh.triangle(0), [0, 1, 2]);
    }

    #[test]
    fn read_ascii_stl_name_with_spaces_success() {
        let mut reader = ::std::io::Cursor::new(
            b"solid foo bar
        facet normal 0.1 0.2 0.3
            outer loop
                vertex 1 2 3
                vertex 4 5 6e-15
                vertex 7 8 9.87654321
            endloop
        endfacet
        endsolid foo bar"
                .to_vec(),
        );
        let maybe_mesh = read_stl(&mut reader);
        assert!(maybe_mesh.is_ok());
        let mesh = maybe_mesh.unwrap();
        assert_eq!(mesh.node_len(), 3);
        assert_eq!(*mesh.node(0), vec3![1.0, 2.0, 3.0]);
        assert_eq!(*mesh.node(1), vec3![4.0, 5.0, 6e-15]);
        assert_eq!(*mesh.node(2), vec3![7.0, 8.0, 9.87654321]);
        assert_eq!(mesh.triangle_len(), 1);
        assert_eq!(*mesh.triangle(0), [0, 1, 2]);
    }

    #[test]
    fn read_ascii_stl_no_header() {
        let mut reader = ::std::io::Cursor::new(
            b"non-solid foobar
        facet normal 1 2 3
            outer loop
                vertex 7 8 9
                vertex 4 5 6
                vertex 7 8 9
            endloop
        endfacet
        endsolid foobar"
                .to_vec(),
        );
        let maybe_mesh = read_stl(&mut reader);
        assert_eq!(
            maybe_mesh.as_ref().err().unwrap().kind(),
            ::std::io::ErrorKind::UnexpectedEof
        );
    }

    #[test]
    fn read_ascii_stl_wrong_number() {
        let mut reader = ::std::io::Cursor::new(
            b"solid foobar
        facet normal 1 2 3
            outer loop
                vertex 7 8 9,
                vertex 4 5 6
                vertex 7 8 9
            endloop
        endfacet
        endsolid foobar"
                .to_vec(),
        );
        let maybe_mesh = read_stl(&mut reader);
        assert_eq!(
            maybe_mesh.as_ref().err().unwrap().kind(),
            ::std::io::ErrorKind::InvalidData
        );
    }

    #[test]
    fn read_ascii_stl_header_unexpected_eof() {
        let mut reader = ::std::io::Cursor::new(b"solid foobar".to_vec());
        let maybe_mesh = read_stl(&mut reader);
        assert_eq!(
            maybe_mesh.as_ref().err().unwrap().kind(),
            ::std::io::ErrorKind::UnexpectedEof
        );
    }

    #[test]
    fn read_ascii_stl_short_facet_header() {
        let mut reader = ::std::io::Cursor::new(
            b"solid foobar
        facet normal
            outer loop
                vertex 7 8 9,
                vertex 4 5 6
            endloop
        endfacet
        endsolid foobar"
                .to_vec(),
        );
        let maybe_mesh = read_stl(&mut reader);
        assert_eq!(
            maybe_mesh.as_ref().err().unwrap().kind(),
            ::std::io::ErrorKind::InvalidData
        );
    }

    #[test]
    fn read_ascii_stl_wrong_facet_header() {
        let mut reader = ::std::io::Cursor::new(
            b"solid foobar
        triangle normal 1 2 3
            outer loop
                vertex 7 8 9,
                vertex 4 5 6
            endloop
        endfacet
        endsolid foobar"
                .to_vec(),
        );
        let maybe_mesh = read_stl(&mut reader);
        assert!(maybe_mesh.is_err());
        assert_eq!(
            maybe_mesh.as_ref().err().unwrap().kind(),
            ::std::io::ErrorKind::InvalidData
        );
    }

    #[test]
    fn read_ascii_stl_vertex_unexpected_eof() {
        let mut reader = ::std::io::Cursor::new(
            b"solid foo bar
        facet normal 0.1 0.2 0.3
            outer loop
                vertex 1 2 3"
                .to_vec(),
        );
        let maybe_mesh = read_stl(&mut reader);
        assert!(maybe_mesh.is_err());
        assert_eq!(
            maybe_mesh.as_ref().err().unwrap().kind(),
            ::std::io::ErrorKind::UnexpectedEof
        );
    }

    #[test]
    fn read_ascii_stl_vertex_not_f64() {
        let mut reader = ::std::io::Cursor::new(
            b"solid foo bar
        facet normal 0.1 0.2 0.3
            outer loop
                vertex 1 2 NaN"
                .to_vec(),
        );
        let maybe_mesh = read_stl(&mut reader);
        assert!(maybe_mesh.is_err());
        assert_eq!(
            maybe_mesh.as_ref().err().unwrap().kind(),
            ::std::io::ErrorKind::InvalidData
        );
    }

    #[test]
    fn read_ascii_stl_not_vertex() {
        let mut reader = ::std::io::Cursor::new(
            b"solid foo bar
        facet normal 0.1 0.2 0.3
            outer loop
                node 1 2 3"
                .to_vec(),
        );
        let maybe_mesh = read_stl(&mut reader);
        assert!(maybe_mesh.is_err());
        assert_eq!(
            maybe_mesh.as_ref().err().unwrap().kind(),
            ::std::io::ErrorKind::InvalidData
        );
    }

    #[test]
    fn read_ascii_stl_vertex_wrong_number() {
        let mut reader = ::std::io::Cursor::new(
            b"solid foo bar
        facet normal 0.1 0.2 0.3
            outer loop
                vertex 1 2"
                .to_vec(),
        );
        let maybe_mesh = read_stl(&mut reader);
        assert!(maybe_mesh.is_err());
        assert_eq!(
            maybe_mesh.as_ref().err().unwrap().kind(),
            ::std::io::ErrorKind::InvalidData
        );
    }

    #[test]
    fn read_ascii_stl_missing_vertex() {
        let mut reader = ::std::io::Cursor::new(
            b"solid foobar
        facet normal 1 2 3
            outer loop
                vertex 7 8 9,
                vertex 4 5 6
            endloop
        endfacet
        endsolid foobar"
                .to_vec(),
        );
        let maybe_mesh = read_stl(&mut reader);
        assert_eq!(
            maybe_mesh.as_ref().err().unwrap().kind(),
            ::std::io::ErrorKind::InvalidData
        );
    }

    #[test]
    fn read_ascii_stl_bunny() {
        let mut reader = ::std::io::Cursor::new(BUNNY_99_ASCII);
        let maybe_mesh = read_stl(&mut reader);
        assert!(maybe_mesh.is_ok());
        let mesh = maybe_mesh.unwrap();
        assert_eq!(mesh.triangle_len(), 99);
    }

    #[test]
    fn read_ascii_stl_bunny_and_write_binary_stl() {
        let mut reader = ::std::io::Cursor::new(BUNNY_99_ASCII);
        let maybe_mesh = read_stl(&mut reader);
        assert!(maybe_mesh.is_ok());
        let mesh = maybe_mesh.unwrap();

        let mut binary_bunny_stl = Vec::<u8>::new();
        let write_result = super::write_stl(&mut binary_bunny_stl, &mesh);
        assert!(write_result.is_ok(), "{:?}", write_result);
        assert_eq!(BUNNY_99.to_vec(), binary_bunny_stl);
    }

    #[test]
    fn read_binary_stl_bunny() {
        let mut reader = ::std::io::Cursor::new(BUNNY_99);
        let maybe_mesh = read_stl(&mut reader);
        assert!(maybe_mesh.is_ok(), "{:?}", maybe_mesh.err());
        let mesh = maybe_mesh.unwrap();
        assert_eq!(mesh.triangle_len(), 99);
    }

    #[test]
    fn read_binary_from_vec() {
        let buffer = BUNNY_99.iter().copied().collect::<Vec<u8>>();
        let mut reader = std::io::Cursor::new(&buffer);
        let maybe_mesh = read_stl(&mut reader);
        assert!(maybe_mesh.is_ok(), "{:?}", maybe_mesh.err());
        let mesh = maybe_mesh.unwrap();
        assert_eq!(mesh.triangle_len(), 99);
    }

    #[test]
    fn read_ascii_and_binary_stl_bunny() {
        let mut binary_reader = ::std::io::Cursor::new(BUNNY_99);
        let maybe_binary_mesh = read_stl(&mut binary_reader);
        assert!(maybe_binary_mesh.is_ok());
        let binary_mesh = maybe_binary_mesh.unwrap();

        let mut ascii_reader = ::std::io::Cursor::new(BUNNY_99_ASCII);
        let maybe_ascii_mesh = read_stl(&mut ascii_reader);
        assert!(maybe_ascii_mesh.is_ok());
        let ascii_mesh = maybe_ascii_mesh.unwrap();

        assert_eq!(ascii_mesh.node_len(), binary_mesh.node_len());
        for n in 0..ascii_mesh.node_len() {
            assert!(relative_eq!(
                ascii_mesh.node(n),
                binary_mesh.node(n),
                epsilon = 0.000001
            ));
        }

        assert_eq!(ascii_mesh.triangle_len(), binary_mesh.triangle_len());
        for t in 0..ascii_mesh.triangle_len() {
            assert_eq!(ascii_mesh.triangle(t), binary_mesh.triangle(t));
        }
    }

    #[test]
    fn read_ascii_stl_tiny_numbers() {
        let mut reader = ::std::io::Cursor::new(
            b"solid ASCII
                  facet normal 8.491608e-001 1.950388e-001 -4.908011e-001
                    outer loop
                    vertex   -8.222098e-001 2.326105e+001 5.724931e-046
                    vertex   -8.811435e-001 2.351764e+001 1.135191e-045
                    vertex   3.688022e+000 2.340444e+001 7.860367e+000
                    endloop
                endfacet
            endsolid"
                .to_vec(),
        );
        let maybe_mesh = read_stl(&mut reader);
        assert!(maybe_mesh.is_ok());
    }

    #[test]
    fn to_file() {
        let mut bunny_path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"));
        bunny_path.push("test_assets/bunny_99.stl");

        let mut test_path = tempdir::TempDir::new("stl_to_file_test")
            .unwrap()
            .into_path();
        test_path.push("test.stl");

        let maybe_mesh = read_stl_file(&bunny_path);
        assert!(maybe_mesh.is_ok(), "{:?}", maybe_mesh.err());
        let mesh = maybe_mesh.unwrap();
        assert_eq!(mesh.triangle_len(), 99);
        let result = write_stl_file(&test_path, &mesh);
        if let Err(error) = result {
            println!("{}", error);
            panic!();
        }
        {
            let maybe_mesh = read_stl_file(&test_path);
            assert!(maybe_mesh.is_ok());
            let mesh = maybe_mesh.unwrap();
            assert_eq!(mesh.triangle_len(), 99);
        }
    }
}
