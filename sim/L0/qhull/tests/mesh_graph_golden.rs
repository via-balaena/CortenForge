//! MuJoCo 3.5.0's mesh graph, int for int, over the cases in
//! `tests/data/mesh_graph_golden.txt` (written by
//! `sim/L0/tests/scripts/gen_qhull_graph_reference.py` from the unfused
//! oracle: each case's float32 points loaded as a mesh with no faces, its
//! `mesh_graph` read back, or MuJoCo's "qhull error").

// A malformed golden line is the test failing: each field panics with what it
// expected.
#![allow(clippy::expect_used)]

use sim_qhull::{Error, mesh_graph};

/// One golden case: its name, `maxhullvert`, points, and MuJoCo's graph
/// (`None` where MuJoCo refused the mesh with "qhull error").
struct Case {
    name: String,
    maxhullvert: i32,
    points: Vec<[f64; 3]>,
    graph: Option<Vec<i32>>,
}

fn cases() -> Vec<Case> {
    let text = include_str!("data/mesh_graph_golden.txt");
    let mut out = Vec::new();
    for line in text.lines().filter(|l| !l.starts_with('#')) {
        let (head, tail) = line
            .split_once(" ; ")
            .expect("a case is '<head> ; <graph>'");
        let mut fields = head.split(' ');
        let name = fields.next().expect("name").to_owned();
        let maxhullvert = fields
            .next()
            .expect("maxhullvert")
            .parse()
            .expect("maxhullvert is an int");
        let count: usize = fields
            .next()
            .expect("count")
            .parse()
            .expect("count is an int");
        let coords: Vec<f64> = fields
            .map(|h| {
                f64::from(f32::from_bits(
                    u32::from_str_radix(h, 16).expect("a coordinate is hex float32 bits"),
                ))
            })
            .collect();
        assert_eq!(coords.len(), 3 * count, "{name}: its point count");
        let points = coords.chunks_exact(3).map(|c| [c[0], c[1], c[2]]).collect();
        let graph = match tail {
            "ERR" => None,
            _ => Some(
                tail.strip_prefix("G ")
                    .expect("a graph starts 'G '")
                    .split(' ')
                    .map(|x| x.parse().expect("a graph entry is an int"))
                    .collect(),
            ),
        };
        out.push(Case {
            name,
            maxhullvert,
            points,
            graph,
        });
    }
    out
}

#[test]
fn the_graph_is_mujocos_on_every_golden_case() {
    let cases = cases();
    // The golden's size, so a truncated or emptied file fails here.
    assert_eq!(cases.len(), 114, "golden cases");
    let mut wrong = Vec::new();
    for case in &cases {
        let got = mesh_graph(&case.points, case.maxhullvert);
        let ok = match (&case.graph, &got) {
            (Some(want), Ok(Some(g))) => &g.to_mujoco() == want,
            (None, Err(Error::Qhull { .. })) => true,
            _ => false,
        };
        if !ok {
            let what = match &got {
                Ok(Some(g)) => format!(
                    "a graph of {} vertices, {} faces",
                    g.vertex_point.len(),
                    g.faces.len()
                ),
                Ok(None) => "no graph".to_owned(),
                Err(e) => e.to_string(),
            };
            wrong.push(format!("{}: {what}", case.name));
        }
    }
    assert!(
        wrong.is_empty(),
        "{} of {} cases differ from MuJoCo:\n{}",
        wrong.len(),
        cases.len(),
        wrong.join("\n")
    );
}

#[test]
fn fewer_than_four_points_make_no_graph() {
    let points = [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]];
    assert_eq!(mesh_graph(&points, -1), Ok(None));
}
