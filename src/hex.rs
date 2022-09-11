enum CoordinateSystem {
    Offset(OffsetCoordinateSystem),
    Cube,
    Axial,

}
enum OffsetCoordinateSystem {
    OddR,
    EvenR,
    OddQ,
    EvenQ
}

struct OffsetCoordinates {
    // Column
    q: isize,
    // Row
    r: isize
}
impl OffsetCoordinates {
    pub const fn new(q:isize,r:isize) -> Self {
        Self {q,r}
    }
}
struct CubeCoordinates {
    q: isize,
    r: isize,
    s: isize
}
impl CubeCoordinates {
    pub const fn new(q:isize,r:isize,s:isize) -> Result<Self,&str> {
        if q + r + s == 0 {
            Ok(Self{q,r,s})
        }
        else {
            Err("q + r + s != 0")
        }
    }
}
struct AxialCoordinates {
    q: isize,
    r: isize
}
impl AxialCoordinates {
    pub const fn new(q:isize,r:isize) -> Self {
        Self {q,r}
    }
    pub const fn s(&self) -> isize {
        -self.q-self.r
    }
}

struct DoubledCoordinates {
    // Column
    q: isize,
    // Row
    r: isize
}
impl OffsetCoordinates {
    pub const fn new(q:isize,r:isize) -> Result<Self,&str> {
        if (q + r) % 2 == 0 {
            Self {q,r}
        }
        else {
            Err("(q + r) % 2 != 0")
        }
    }
}