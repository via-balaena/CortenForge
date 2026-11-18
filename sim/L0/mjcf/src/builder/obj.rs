//! An OBJ file read as MuJoCo 3.5.0 reads one.
//!
//! MuJoCo hands an OBJ file's bytes to tinyobjloader (`mjCMesh::LoadOBJ`,
//! `user_mesh.cc:1060-1130`) at the commit it pins, `1421a10d`
//! (`cmake/MujocoDependencies.cmake:25`), built in single precision with its
//! default configuration, which triangulates every polygon; this module
//! reads as that build does when compiled without fused multiply-adds
//! (`-ffp-contract=off`), as the MuJoCo this crate's goldens are measured on
//! is. MuJoCo keeps every vertex the file holds and the faces of the file's
//! first shape only. [`read_obj`] ports what of tinyobjloader's `LoadObj`
//! (`tiny_obj_loader.h:2173-2680`) decides those: its lines, numbers and
//! indices, the shapes its groups make, and its ear clipping. A normal, a
//! texture coordinate, a tag or a smoothing group is not read; a material
//! only as far as its change exports the faces read so far, and a
//! texture-coordinate or normal index only as far as an index of 0 fails the
//! file.

/// The vertices of an OBJ file, three floats each, and its first shape's
/// faces, three vertex ids each, as MuJoCo 3.5.0's `LoadOBJ` hands them to
/// `ProcessVertices` (`user_mesh.cc:1077-1130`).
///
/// A file tinyobjloader refuses is refused in MuJoCo's words, "could not
/// parse OBJ file", naming the file as written (`file`): an index of 0 in a
/// face, line or point, its vertex, texture coordinate or normal slot alike
/// (`fixIndex`, `tiny_obj_loader.h:748-769`). Nothing else is refused here;
/// MuJoCo's later checks see the vertices and faces.
pub fn read_obj(bytes: &[u8], file: &str) -> Result<(Vec<f32>, Vec<i64>), String> {
    let mut reader = Reader::default();
    for line in lines(bytes) {
        reader
            .line(line)
            .ok_or_else(|| format!("could not parse OBJ file '{file}'"))?;
    }
    // At the end tinyobjloader keeps the shape where its group held anything
    // or it holds faces already (`:2656-2665`): one it does not keep holds no
    // faces, so it is kept here either way.
    reader.export();
    reader.push();
    let faces = reader.first.unwrap_or_default();
    Ok((reader.vertices, faces.into_iter().map(i64::from).collect()))
}

/// The lines of a file as tinyobjloader's `safeGetline` splits them
/// (`tiny_obj_loader.h:708-740`): at `\n`, `\r` or `\r\n`, the last line
/// needing none; each ending at its first NUL, where the C string tinyobjloader
/// reads it as ends.
fn lines(bytes: &[u8]) -> impl Iterator<Item = &[u8]> {
    let mut rest = bytes;
    std::iter::from_fn(move || {
        if rest.is_empty() {
            return None;
        }
        let end = rest
            .iter()
            .position(|&b| b == b'\n' || b == b'\r')
            .unwrap_or(rest.len());
        let line = &rest[..end];
        let next = match (rest.get(end), rest.get(end + 1)) {
            (Some(b'\r'), Some(b'\n')) => end + 2,
            (Some(_), _) => end + 1,
            (None, _) => end,
        };
        rest = &rest[next..];
        Some(line.split(|&b| b == 0).next().unwrap_or(line))
    })
}

/// What tinyobjloader's `LoadObj` holds as it reads a file
/// (`tiny_obj_loader.h:2178-2205`).
#[derive(Default)]
struct Reader {
    /// Every vertex read so far, three floats each (`v`).
    vertices: Vec<f32>,
    /// The group being read (`prim_group`).
    group: Group,
    /// The shape its groups are exported to (`shape`).
    shape: Shape,
    /// Whether an `mtllib` line has had tinyobjloader read MuJoCo's empty
    /// material text, which names one material `""` (`LoadMtl`'s last
    /// material, `:2048-2051`).
    empty_material: bool,
    /// Whether the material a `usemtl` line last chose is `""`, where it is
    /// one; else no material is chosen (`material`, −1 at the start).
    material_chosen: bool,
    /// The first shape pushed, its faces: the only faces MuJoCo keeps
    /// (`GetShapes()[0]`, `user_mesh.cc:1082`).
    first: Option<Vec<i32>>,
}

/// One group's faces, lines and points (`PrimGroup`, `:688-704`), a face
/// holding its vertex ids (`v_idx`): zero-based, negative where a relative
/// index counts back past the first vertex. A line or a point holds only how
/// many ids it names.
#[derive(Default)]
struct Group {
    faces: Vec<Vec<i32>>,
    lines: Vec<usize>,
    points: Vec<usize>,
}

/// A shape's triangles, three vertex ids each, and how many ids its lines
/// and points name (`shape_t`).
#[derive(Default)]
struct Shape {
    faces: Vec<i32>,
    line_ids: usize,
    point_ids: usize,
}

/// The byte at `at` of a line, 0 past its end, as a C string reads it.
fn byte(line: &[u8], at: usize) -> u8 {
    line.get(at).copied().unwrap_or(0)
}

/// tinyobjloader's `IS_SPACE`.
const fn is_space(b: u8) -> bool {
    b == b' ' || b == b'\t'
}

/// `at` moved past the bytes of `set`, as C's `strspn` moves it.
fn skip(line: &[u8], at: usize, set: &[u8]) -> usize {
    at + line
        .get(at..)
        .unwrap_or_default()
        .iter()
        .take_while(|b| set.contains(b))
        .count()
}

/// `at` moved to the first byte of `set` or the line's end, as C's
/// `strcspn` moves it.
fn skip_to(line: &[u8], at: usize, set: &[u8]) -> usize {
    at + line
        .get(at..)
        .unwrap_or_default()
        .iter()
        .take_while(|b| !set.contains(b))
        .count()
}

impl Reader {
    /// One line, as `LoadObj`'s loop reads it (`:2208-2622`), in its order of
    /// tests. `None` where tinyobjloader fails the file.
    fn line(&mut self, line: &[u8]) -> Option<()> {
        let at = skip(line, 0, b" \t");
        let [c0, c1] = [0, 1].map(|k| byte(line, at + k));
        if c0 == 0 || c0 == b'#' {
            return Some(());
        }
        if c0 == b'v' && is_space(c1) {
            // `parseVertexWithColor` (`:991-1009`): x, y and z, each 0 where
            // its token does not read; a colour after them is not kept here.
            let mut at = at + 2;
            for _ in 0..3 {
                self.vertices.push(parse_real(line, &mut at));
            }
        } else if (c0 == b'l' || c0 == b'p') && is_space(c1) {
            // No space is skipped before the first index (`:2281`, `:2313`).
            let ids = self.indices(line, at + 2)?.len();
            if c0 == b'l' {
                self.group.lines.push(ids);
            } else {
                self.group.points.push(ids);
            }
        } else if c0 == b'f' && is_space(c1) {
            let start = skip(line, at + 2, b" \t");
            let face = self.indices(line, start)?;
            self.group.faces.push(face);
        } else if line.get(at..at + 6) == Some(b"usemtl") {
            // `:2383-2409`: a material change exports the group's faces to the
            // shape, which stays open.
            let name = skip(line, at + 6, b" \t");
            let empty_name = skip_to(line, name, b" \t\r") == name;
            let chosen = empty_name && self.empty_material;
            if chosen != self.material_chosen {
                self.export();
                self.group.faces.clear();
                self.material_chosen = chosen;
            }
        } else if line.get(at..at + 6) == Some(b"mtllib") && is_space(byte(line, at + 6)) {
            // `:2411-2460`: a file name after it has tinyobjloader read
            // MuJoCo's empty material text (`SplitString` finds none in an
            // empty rest).
            if byte(line, at + 7) != 0 {
                self.empty_material = true;
            }
        } else if (c0 == b'g' || c0 == b'o') && is_space(c1) {
            // `:2462-2536`: the group is exported and the shape pushed if it
            // holds faces; after `o`, lines or points too.
            self.export();
            let s = &self.shape;
            let keep = !s.faces.is_empty() || (c0 == b'o' && (s.line_ids > 0 || s.point_ids > 0));
            if keep {
                self.push();
            }
            self.shape = Shape::default();
            self.group = Group::default();
        }
        // A normal (`vn`), a texture coordinate (`vt`), a tag (`t`), a
        // smoothing group (`s`) or any other line moves nothing `read_obj`
        // returns.
        Some(())
    }

    /// The vertex ids of a face, line or point from `at`, as `parseTriple`
    /// reads each (`:1078-1129`) and `LoadObj` steps between them
    /// (`:2349-2378`). `None` where an index is 0.
    fn indices(&self, line: &[u8], mut at: usize) -> Option<Vec<i32>> {
        let nv = i32::try_from(self.vertices.len() / 3).unwrap_or(i32::MAX);
        // `fixIndex` fails a texture-coordinate or normal index only where it
        // is 0; the id it makes is not among what `read_obj` returns.
        let slot = |at: usize| (atoi(line, at) != 0).then_some(());
        let mut ids = Vec::new();
        while !matches!(byte(line, at), 0 | b'\r' | b'\n') {
            let v = fix_index(atoi(line, at), nv)?;
            at = skip_to(line, at, b"/ \t\r");
            if byte(line, at) == b'/' {
                at += 1;
                if byte(line, at) == b'/' {
                    at += 1;
                    slot(at)?;
                    at = skip_to(line, at, b"/ \t\r");
                } else {
                    slot(at)?;
                    at = skip_to(line, at, b"/ \t\r");
                    if byte(line, at) == b'/' {
                        at += 1;
                        slot(at)?;
                        at = skip_to(line, at, b"/ \t\r");
                    }
                }
            }
            ids.push(v);
            at = skip(line, at, b" \t\r");
        }
        Some(ids)
    }

    /// The group's faces triangulated into the shape, and its lines' and
    /// points' ids counted (`exportGroupsToShape`, `:1343-1637`), each
    /// polygon against the vertices read so far.
    fn export(&mut self) {
        for face in &self.group.faces {
            triangulate(face, &self.vertices, &mut self.shape.faces);
        }
        self.shape.line_ids += self.group.lines.iter().sum::<usize>();
        self.shape.point_ids += self.group.points.iter().sum::<usize>();
    }

    /// The shape pushed: the first one pushed keeps its faces.
    fn push(&mut self) {
        if self.first.is_none() {
            self.first = Some(std::mem::take(&mut self.shape.faces));
        }
    }
}

/// C's `atoi` from `at`, as Apple's libc and glibc make it in the C locale:
/// `(int)strtol`, so the C locale's spaces skipped, a sign, digits, a value
/// past a 64-bit `long` held at its bound and then cut to an `int`'s 32 bits.
#[allow(clippy::cast_possible_truncation)] // the cut is `(int)`'s
fn atoi(line: &[u8], at: usize) -> i32 {
    let mut at = skip(line, at, b" \t\n\x0b\x0c\r");
    let negative = byte(line, at) == b'-';
    if matches!(byte(line, at), b'+' | b'-') {
        at += 1;
    }
    let mut magnitude: i128 = 0;
    while byte(line, at).is_ascii_digit() {
        magnitude = (magnitude * 10 + i128::from(byte(line, at) - b'0')).min(i128::from(u64::MAX));
        at += 1;
    }
    let value = if negative { -magnitude } else { magnitude };
    value.clamp(i128::from(i64::MIN), i128::from(i64::MAX)) as i32
}

/// tinyobjloader's `fixIndex` (`:748-769`): a one-based index made zero-based,
/// a negative one counted back from the `n` read so far, 0 refused.
const fn fix_index(index: i32, n: i32) -> Option<i32> {
    match index {
        0 => None,
        i if i > 0 => Some(i - 1),
        i => Some(n.wrapping_add(i)),
    }
}

/// One coordinate from `at`, as `parseReal` reads it with its default 0
/// (`:940-948`): the token up to a space, tab or `\r`, read by
/// [`try_parse_double`] and cast to a float; 0 where it does not read.
#[allow(clippy::cast_possible_truncation)] // the cast is `static_cast<float>`'s
fn parse_real(line: &[u8], at: &mut usize) -> f32 {
    let start = skip(line, *at, b" \t");
    let end = skip_to(line, start, b" \t\r");
    *at = end;
    try_parse_double(line.get(start..end).unwrap_or_default()).unwrap_or(0.0) as f32
}

/// tinyobjloader's `tryParseDouble` (`:814-938`) over one token, `None` where
/// it fails. Its digits are summed in double precision, a fraction's by
/// `0.1`, `0.01`, … ([`ten_to_minus`] from the eighth), and an exponent
/// applied as `ldexp(m * pow(5, e), e)` ([`five_to`], [`ldexp`]), so its
/// result is not always the double nearest the text.
fn try_parse_double(s: &[u8]) -> Option<f64> {
    const POW_LUT: [f64; 8] = [
        1.0,
        0.1,
        0.01,
        0.001,
        0.0001,
        0.00001,
        0.000_001,
        0.000_000_1,
    ];
    // Past the token C reads its separator, never a digit, sign, `.` or `e`.
    let at = |i: usize| byte(s, i);
    let digit = |b: u8| f64::from(b - b'0');
    if s.is_empty() {
        return None;
    }
    let mut curr = 0;
    let negative = at(0) == b'-';
    let leading_dot = if matches!(at(0), b'+' | b'-') {
        curr = 1;
        curr != s.len() && at(1) == b'.'
    } else if at(0).is_ascii_digit() {
        false
    } else if at(0) == b'.' {
        true
    } else {
        return None;
    };
    let mut mantissa = 0.0_f64;
    let mut exponent: i32 = 0;
    if !leading_dot {
        let first = curr;
        while curr != s.len() && at(curr).is_ascii_digit() {
            mantissa *= 10.0;
            mantissa += digit(at(curr));
            curr += 1;
        }
        if curr == first {
            return None;
        }
    }
    'assemble: {
        if curr == s.len() {
            break 'assemble;
        }
        if at(curr) == b'.' {
            curr += 1;
            let mut read: i32 = 1;
            while curr != s.len() && at(curr).is_ascii_digit() {
                let scale = usize::try_from(read)
                    .ok()
                    .and_then(|r| POW_LUT.get(r))
                    .copied()
                    .unwrap_or_else(|| ten_to_minus(read));
                mantissa += digit(at(curr)) * scale;
                read = read.saturating_add(1);
                curr += 1;
            }
        } else if !matches!(at(curr), b'e' | b'E') {
            break 'assemble;
        }
        if curr == s.len() || !matches!(at(curr), b'e' | b'E') {
            break 'assemble;
        }
        curr += 1;
        let exp_negative = curr != s.len() && at(curr) == b'-';
        if curr != s.len() && matches!(at(curr), b'+' | b'-') {
            curr += 1;
        } else if !at(curr).is_ascii_digit() {
            return None;
        }
        let first = curr;
        while curr != s.len() && at(curr).is_ascii_digit() {
            exponent = exponent
                .wrapping_mul(10)
                .wrapping_add(i32::from(at(curr) - b'0'));
            curr += 1;
        }
        if exp_negative {
            exponent = exponent.wrapping_neg();
        }
        if curr == first {
            return None;
        }
    }
    let value = if exponent == 0 {
        mantissa
    } else {
        ldexp(mantissa * five_to(exponent), exponent)
    };
    Some(if negative { -value } else { value })
}

/// The powers tinyobjloader's `std::pow` calls take: 10^-n for n from 0 to
/// 343 (`__exp10(-n)` in MuJoCo's build, which clang makes of
/// `std::pow(10.0, -n)`) and 5^e for e from −470 to 450, each the double
/// nearest it: Rust's parse rounds a decimal text once. Past those bounds
/// each is 0 or infinite.
struct Powers {
    ten_to_minus: Vec<f64>,
    five_to: Vec<f64>,
}

/// The least e [`Powers`] holds 5^e for.
const FIVE_FROM: i32 = -470;

fn powers() -> &'static Powers {
    static POWERS: std::sync::OnceLock<Powers> = std::sync::OnceLock::new();
    POWERS.get_or_init(|| {
        let parse = |text: String| text.parse::<f64>().unwrap_or(f64::NAN);
        Powers {
            ten_to_minus: (0..=343).map(|n| parse(format!("1e-{n}"))).collect(),
            // 5^e for e < 0 is 2^-e · 10^e.
            five_to: (FIVE_FROM..=450)
                .map(|e| {
                    let (base, exp10) = if e < 0 { (2, e) } else { (5, 0) };
                    parse(format!("{}e{exp10}", decimal_power(base, e.unsigned_abs())))
                })
                .collect(),
        }
    })
}

/// 10^-n as [`Powers`] holds it, 0 past 343.
fn ten_to_minus(n: i32) -> f64 {
    let at = usize::try_from(n).ok();
    at.and_then(|at| powers().ten_to_minus.get(at))
        .copied()
        .unwrap_or(0.0)
}

/// 5^e as [`Powers`] holds it, 0 below −470 and infinite above 450.
fn five_to(e: i32) -> f64 {
    if e < FIVE_FROM {
        return 0.0;
    }
    let at = usize::try_from(i64::from(e) - i64::from(FIVE_FROM)).ok();
    at.and_then(|at| powers().five_to.get(at))
        .copied()
        .unwrap_or(f64::INFINITY)
}

/// `base^k` in decimal digits.
fn decimal_power(base: u64, k: u32) -> String {
    const LIMB: u64 = 1_000_000_000;
    let mut limbs: Vec<u64> = vec![1]; // base 10^9, least first
    for _ in 0..k {
        let mut carry = 0;
        for limb in &mut limbs {
            let x = *limb * base + carry;
            *limb = x % LIMB;
            carry = x / LIMB;
        }
        if carry > 0 {
            limbs.push(carry);
        }
    }
    let mut digits = limbs.last().map_or_else(String::new, ToString::to_string);
    digits.extend(limbs.iter().rev().skip(1).map(|limb| format!("{limb:09}")));
    digits
}

/// `x · 2^n` rounded once, as C's `ldexp` gives it: musl's `scalbn`, which
/// steps a large `n` by 2^1023 and a small one by 2^-969 so that a result
/// below the normal range is rounded only by its last product.
fn ldexp(x: f64, n: i32) -> f64 {
    let two_1023 = f64::from_bits(0x7fe0_0000_0000_0000);
    let two_minus_969 = f64::from_bits(0x0360_0000_0000_0000); // 2^-1022 · 2^53
    let (mut y, mut n) = (x, n);
    if n > 1023 {
        y *= two_1023;
        n -= 1023;
        if n > 1023 {
            y *= two_1023;
            n = (n - 1023).min(1023);
        }
    } else if n < -1022 {
        y *= two_minus_969;
        n += 1022 - 53;
        if n < -1022 {
            y *= two_minus_969;
            n = (n + 1022 - 53).max(-1022);
        }
    }
    let bits = u64::try_from(0x3ff + n).unwrap_or_default() << 52;
    y * f64::from_bits(bits)
}

/// One face's triangles, three vertex ids each, appended to `out` as
/// tinyobjloader ear-clips it (`exportGroupsToShape`'s triangulation,
/// `:1360-1574`), in single precision: a face of fewer than 3 vertices
/// dropped, a triangle kept as written; a polygon projected onto the two axes
/// tinyobjloader picks (`:1372-1414`), then its ears cut where its signed
/// area's sign and the other vertices allow, losing whatever it cannot cut in
/// as many tries as it has vertices. A vertex id that names no vertex reads
/// as the origin, or is passed over where tinyobjloader passes it over, and
/// is kept in the triangle.
fn triangulate(face: &[i32], coords: &[f32], out: &mut Vec<i32>) {
    let sides = face.len();
    if sides <= 3 {
        if sides == 3 {
            out.extend_from_slice(face);
        }
        return;
    }
    let vertex = |id: i32| -> Option<[f32; 3]> {
        let at = usize::try_from(id).ok()?.checked_mul(3)?;
        Some([*coords.get(at)?, *coords.get(at + 1)?, *coords.get(at + 2)?])
    };
    let mut axes = [1, 2];
    for k in 0..sides {
        let (Some(p0), Some(p1), Some(p2)) = (
            vertex(face[k]),
            vertex(face[(k + 1) % sides]),
            vertex(face[(k + 2) % sides]),
        ) else {
            continue;
        };
        let e0 = [p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]];
        let e1 = [p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2]];
        let cx = (e0[1] * e1[2] - e0[2] * e1[1]).abs();
        let cy = (e0[2] * e1[0] - e0[0] * e1[2]).abs();
        let cz = (e0[0] * e1[1] - e0[1] * e1[0]).abs();
        if cx > f32::EPSILON || cy > f32::EPSILON || cz > f32::EPSILON {
            if !(cx > cy && cx > cz) {
                axes[0] = 0;
                if cz > cx && cz > cy {
                    axes[1] = 1;
                }
            }
            break;
        }
    }
    let [across, up] = axes;
    let mut area = 0.0_f32;
    for k in 0..sides {
        if let (Some(p0), Some(p1)) = (vertex(face[k]), vertex(face[(k + 1) % sides])) {
            area += (p0[across] * p1[up] - p0[up] * p1[across]) * 0.5;
        }
    }
    let planar = |id: i32| vertex(id).map(|p| [p[across], p[up]]);
    let mut rest = face.to_vec();
    let mut guess = 0;
    let mut tries = sides;
    let mut previous = sides;
    while rest.len() > 3 && tries > 0 {
        let left = rest.len();
        if guess >= left {
            guess -= left;
        }
        if previous == left {
            tries -= 1;
        } else {
            previous = left;
            tries = left;
        }
        let ids: [i32; 3] = std::array::from_fn(|k| rest[(guess + k) % left]);
        let corners = ids.map(|id| planar(id).unwrap_or([0.0, 0.0]));
        let (vx, vy) = (corners.map(|c| c[0]), corners.map(|c| c[1]));
        let (e0x, e0y) = (vx[1] - vx[0], vy[1] - vy[0]);
        let (e1x, e1y) = (vx[2] - vx[1], vy[2] - vy[1]);
        let cross = e0x * e1y - e0y * e1x;
        let reflex = cross * area < 0.0;
        let holds_another = || {
            (3..left).any(|k| {
                planar(rest[(guess + k) % left]).is_some_and(|[tx, ty]| pnpoly(vx, vy, tx, ty))
            })
        };
        if reflex || holds_another() {
            guess += 1;
            continue;
        }
        out.extend_from_slice(&ids);
        rest.remove((guess + 1) % left);
    }
    if rest.len() == 3 {
        out.extend_from_slice(&rest);
    }
}

/// Whether a point lies in a triangle by the crossing test tinyobjloader
/// takes from W. Randolph Franklin's PNPOLY (`pnpoly`, `:1329-1340`), in
/// single precision.
fn pnpoly(vx: [f32; 3], vy: [f32; 3], tx: f32, ty: f32) -> bool {
    let mut inside = false;
    let mut j = 2;
    for i in 0..3 {
        if (vy[i] > ty) != (vy[j] > ty)
            && tx < (vx[j] - vx[i]) * (ty - vy[i]) / (vy[j] - vy[i]) + vx[i]
        {
            inside = !inside;
        }
        j = i;
    }
    inside
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used, clippy::panic)]
mod tests {
    use super::{read_obj, try_parse_double};

    /// The bytes of a case as the golden escapes them on one line.
    fn unescape(text: &str) -> Vec<u8> {
        let mut bytes = Vec::new();
        let mut chars = text.bytes();
        while let Some(b) = chars.next() {
            if b != b'\\' {
                bytes.push(b);
                continue;
            }
            match chars.next() {
                Some(b'n') => bytes.push(b'\n'),
                Some(b'r') => bytes.push(b'\r'),
                Some(b't') => bytes.push(b'\t'),
                Some(b'0') => bytes.push(0),
                Some(b'\\') => bytes.push(b'\\'),
                Some(b'x') => {
                    let hex = [chars.next().unwrap(), chars.next().unwrap()];
                    bytes.push(u8::from_str_radix(std::str::from_utf8(&hex).unwrap(), 16).unwrap());
                }
                other => panic!("escape {other:?}"),
            }
        }
        bytes
    }

    /// tinyobjloader's own reading of each OBJ text in `tests/assets/obj/
    /// tinyobjloader.txt` (its header says how it was made) is
    /// [`read_obj`]'s: whether the file reads, every coordinate (by its float
    /// bits, a NaN as a NaN) and every id of the first shape's faces.
    #[test]
    fn every_case_reads_as_tinyobjloader_reads_it() {
        let path = concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/tests/assets/obj/tinyobjloader.txt"
        );
        let golden = std::fs::read_to_string(path).unwrap();
        let mut cases = 0;
        for case in golden.split("\ncase ").skip(1) {
            let mut lines = case.lines();
            let name = lines.next().unwrap();
            let bytes = unescape(lines.next().unwrap().strip_prefix("obj ").unwrap());
            let valid = lines.next().unwrap() == "valid 1";
            let read = read_obj(&bytes, name);
            assert_eq!(read.is_ok(), valid, "{name}");
            if let Ok((vertices, faces)) = read {
                let words = |line: Option<&str>, key: &str| -> Vec<String> {
                    let rest = line.unwrap().strip_prefix(key).unwrap();
                    rest.split_whitespace().map(str::to_string).collect()
                };
                // Rust leaves a NaN's sign to the platform (`f32`'s "NaN bit
                // patterns"), so a NaN is compared as a NaN, every other float
                // by its bits.
                let bits = |x: f32| {
                    if x.is_nan() {
                        f32::NAN.to_bits()
                    } else {
                        x.to_bits()
                    }
                };
                let want: Vec<u32> = words(lines.next(), "v")
                    .iter()
                    .map(|w| bits(f32::from_bits(u32::from_str_radix(w, 16).unwrap())))
                    .collect();
                let got: Vec<u32> = vertices.iter().map(|&x| bits(x)).collect();
                assert_eq!(got, want, "{name}");
                let want: Vec<i64> = words(lines.next(), "f")
                    .iter()
                    .map(|w| w.parse().unwrap())
                    .collect();
                assert_eq!(faces, want, "{name}");
            }
            cases += 1;
        }
        assert_eq!(cases, 140);
    }

    /// On MuJoCo's platform, macOS on arm64, the powers are the libm's
    /// (`pow`, or `__exp10`, which LLVM may make of a power of 10, both
    /// measured) at every exponent a token can bring, but 5^-441, a subnormal
    /// one bit off: no float read shows it, as a value at that exponent is
    /// below the float range for every mantissa up to the largest double.
    #[cfg(all(target_os = "macos", target_arch = "aarch64"))]
    #[test]
    #[allow(clippy::cast_possible_truncation)]
    fn the_powers_are_the_platform_libms() {
        use super::{five_to, ldexp, ten_to_minus};
        for n in 0..=400 {
            let libm = 10.0_f64.powf(-f64::from(n));
            assert_eq!(ten_to_minus(n).to_bits(), libm.to_bits(), "10^-{n}");
        }
        let differ: Vec<i32> = (-1100..=1100)
            .filter(|&e| five_to(e).to_bits() != 5.0_f64.powf(f64::from(e)).to_bits())
            .collect();
        assert_eq!(differ, [-441]);
        for m in [f64::MAX, 1.0] {
            let libm = ldexp(m * 5.0_f64.powf(-441.0), -441) as f32;
            assert_eq!(
                (ldexp(m * five_to(-441), -441) as f32).to_bits(),
                libm.to_bits()
            );
            assert_eq!(libm.to_bits(), 0.0_f32.to_bits());
        }
    }

    /// A token is read to the first byte that ends it; one tinyobjloader
    /// cannot read is `None`, which its caller makes 0.
    #[test]
    fn a_token_is_read_as_far_as_tinyobjloader_reads_it() {
        let read = |t: &str| try_parse_double(t.as_bytes());
        assert_eq!(read("1e5.5"), Some(100_000.0));
        assert_eq!(read("1.5abc"), Some(1.5));
        assert_eq!(read(".").map(f64::to_bits), Some(0.0_f64.to_bits()));
        assert_eq!(read("-.").map(f64::to_bits), Some((-0.0_f64).to_bits()));
        for unread in ["nan", "inf", "1.5e", "1e+", "+", "-", "x1", ""] {
            assert_eq!(read(unread), None, "{unread}");
        }
        assert!(read("0e1000").is_some_and(f64::is_nan));
    }
}
