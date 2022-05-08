use bevy::prelude::*;
use bevy_prototype_lyon::prelude::*;
use rand::{thread_rng, Rng};
use std::{
    collections::{HashMap, HashSet},
    sync::atomic::AtomicUsize,
};

const UNIT_SPRITE_SINGLE: &'static str = "units/enemy1idle1.png";
const UNIT_SPRITE_PATH: &'static str = "./assets/units";
const ENEMY_SPRITE_SINGLE: &'static str = "enemy/enemy3idle1.png";
const ENEMY_SPRITE_PATH: &'static str = "./assets/enemy";

/// Fill colour of obstructed tiles on map.
const OBSTRUCTION_HEX_COLOUR: Color = Color::rgba(1f32, 1f32, 1f32, 1f32);
/// Outline colour of obstructed tiles on the map.
const OBSTACLE_HEX_OUTLINE_COLOUR: Color = Color::rgba(0f32, 0f32, 0f32, 0.8f32);
/// Colour of hexagons used to form the map.
const HEX_COLOR: Color = Color::rgba(0.6f32, 0.6f32, 0.6f32, 1f32);

/// An hexagon grid.
#[derive(Debug, Clone)]
pub struct HexGrid<T: std::fmt::Debug> {
    pub data: Vec<T>,
    pub width: usize,
    pub height: usize,
    /// Logical pixel bounds of hex grid.
    pub logical_pixel_bounds: [std::ops::Range<f32>; 2],
    /// A mapping of colour highlights to their respective entity.
    pub highlights: HashMap<Colour, Entity>,
}

#[derive(Debug, Component)]
pub struct Animation {
    timer: Timer,
    frames: std::iter::Cycle<std::vec::IntoIter<bevy::prelude::Handle<Image>>>,
}
impl Animation {
    pub fn from_dir(path: &str, asset_server: &AssetServer) -> std::io::Result<Self> {
        let path = std::path::Path::new(&path);
        assert!(path.is_dir());

        let mut frames = Vec::new();

        // Orders paths alphabetically
        let mut paths = path
            .read_dir()?
            .map(|entry| entry.unwrap().path())
            .collect::<Vec<_>>();
        paths.sort();
        // println!("paths: {:?}",paths);

        for image_path in paths {
            assert_eq!(image_path.extension().unwrap(), "png");
            let relative_path = format!("../{}", image_path.display());
            // println!("relative_path: {}",relative_path);

            frames.push(asset_server.load(&relative_path))
        }
        assert!(!frames.is_empty());

        // To avoid animation syncing and looking weird we skip a random number of initial frames.
        let mut rng = thread_rng();
        let skip = rng.gen_range(0..frames.len());
        let mut iter = frames.into_iter().cycle();
        iter.nth(skip);

        Ok(Self {
            timer: Timer::from_seconds(0.3f32, true),
            frames: iter,
        })
    }
    pub fn tick(&mut self, step: bevy::utils::Duration) -> Option<Handle<Image>> {
        if self.timer.tick(step).just_finished() {
            self.frames.next()
        } else {
            None
        }
    }
}

impl HexGrid<HexItem> {
    pub fn add_enemy(
        &mut self,
        commands: &mut Commands,
        component: EnemyUnit,
        index: [usize; 2],
        asset_server: &AssetServer,
    ) {
        #[cfg(debug_assertions)]
        println!("add_enemy() started");
        let [x, y] = self.logical_pixels(index).unwrap();
        let new_entity = commands
            .spawn_bundle(SpriteBundle {
                transform: Transform {
                    translation: Vec3::from((x, y, 1f32)),
                    scale: Vec3::new(1f32, 1f32, 1f32),
                    ..Default::default()
                },
                // sprite: Sprite {
                //     color: Color::rgb(1., 1., 1.),
                //     ..Default::default()
                // },
                texture: asset_server.load(ENEMY_SPRITE_SINGLE),
                ..Default::default()
            })
            .insert(component)
            .insert(Animation::from_dir(ENEMY_SPRITE_PATH, asset_server).unwrap())
            .id();
        self[index] = HexItem::Enemy(new_entity);

        #[cfg(debug_assertions)]
        println!("add_enemy() finished");
    }

    pub fn add_unit(
        &mut self,
        commands: &mut Commands,
        index: [usize; 2],
        asset_server: &AssetServer,
    ) {
        #[cfg(debug_assertions)]
        println!("add_unit() started");
        let [x, y] = self.logical_pixels(index).unwrap();

        let unit = Unit::default(commands, asset_server, [x, y]);

        self[index] = HexItem::Entity(unit);

        #[cfg(debug_assertions)]
        println!("add_unit() finished");
    }
    pub fn add_obstruction(
        &mut self,
        commands: &mut Commands,
        index: [usize; 2],
        asset_server: &AssetServer,
    ) {
        let [x, y] = self.logical_pixels(index).unwrap();

        // Elements higher up on the y axis we want to be behind elements lower down, thus we subtract this from the z dimension.
        // let z_adj = y as f32 / self.logical_pixel_bounds[1].end;
        // println!("y: {:?}, z_adj: {}", y, z_adj);

        crate::spawn_hex(
            [x, y],
            OBSTRUCTION_HEX_COLOUR,
            OBSTACLE_HEX_OUTLINE_COLOUR,
            commands,
        );
        // commands.spawn_bundle(SpriteBundle {
        //     transform: Transform {
        //         translation: Vec3::from((x, y, 2f32 - z_adj)),
        //         scale: Vec3::new(0.5f32, 0.5f32, 1f32),
        //         ..Default::default()
        //     },
        //     // sprite: Sprite {
        //     //     color: Color::rgb(1., 1., 1.),
        //     //     ..Default::default()
        //     // },
        //     texture: asset_server.load("rock.png"),
        //     ..Default::default()
        // });

        self[index] = HexItem::Obstruction;
    }
    /// Returns a 2d vec of reachable indices from the given index (where `vec[x][2]` denotes the an index reachable in `x` steps).
    pub fn reachable(&self, start: [usize; 2], movement: usize) -> Vec<Vec<[usize; 2]>> {
        let mut visited = HashSet::new();
        visited.insert(start);
        let mut fringes = vec![vec![start]];
        for k in 0..movement {
            let mut temp = Vec::new();
            for hex in fringes[k].iter() {
                for dir in 0..6 {
                    let neighbor_opt = self.neighbor(*hex, dir);
                    // If neighbor exists
                    if let Some(neighbor) = neighbor_opt {
                        // If neighbor has not been visited and is not blocked
                        if !visited.contains(&neighbor) && self[neighbor].is_empty() {
                            visited.insert(neighbor);
                            temp.push(neighbor);
                        }
                    }
                }
            }
            fringes.push(temp);
        }
        fringes
    }
}
impl<T: std::fmt::Debug + Default + Copy> HexGrid<T> {
    /// `HexGrid::<i32>::new(4,2)` produces:
    /// ```text
    ///  _   _
    /// ╱0╲_╱0╲_
    /// ╲_╱0╲_╱0╲
    /// ╱0╲_╱0╲_╱
    /// ╲_╱0╲_╱0╲
    ///   ╲_╱ ╲_╱
    /// ```
    pub fn new(width: usize, height: usize) -> Self {
        let mut this = Self {
            data: vec![Default::default(); width * height],
            width,
            height,
            logical_pixel_bounds: Default::default(),
            highlights: HashMap::new(),
        };
        this.logical_pixel_bounds = {
            let [xs, ys] = this.unchecked_logical_pixels([0, 0]);
            let [xe, ye] = this.unchecked_logical_pixels([this.width - 1, this.height - 1]);
            [xs..xe, ys..ye]
        };
        this
    }
}
impl<T: std::fmt::Debug> HexGrid<T> {
    /// Gets logical pixel coordinates of center of hex of a given index, returning `None` if the
    ///  given coordinates do not correspond to a hex in the grid.
    pub fn logical_pixels(&self, index: [usize; 2]) -> Option<[f32; 2]> {
        if self.contains_index(index) {
            Some(self.unchecked_logical_pixels(index))
        } else {
            None
        }
    }
    /// Gets logical pixel coordinates of center of hex of a given index.
    pub fn unchecked_logical_pixels(&self, [x, y]: [usize; 2]) -> [f32; 2] {
        [
            -(crate::HEX_WIDTH * self.width as f32 / 2f32) + crate::HEX_WIDTH * x as f32,
            -(crate::HEX_HEIGHT * self.height as f32 / 2f32)
                + (crate::HEX_HEIGHT * y as f32)
                + ((x % 2) as f32 * crate::HEX_HEIGHT / 2f32),
        ]
    }
    /// Gets index of logical pixel coordinates in hex grid, if the the given coordinates are within the grid.
    pub fn index(&self, [x, y]: [f32; 2]) -> Option<[usize; 2]> {
        if self.logical_pixel_bounds[0].contains(&x) && self.logical_pixel_bounds[1].contains(&y) {
            // Horizontal index
            let i = ((x + crate::HEX_WIDTH / 2f32) + (crate::HEX_WIDTH * self.width as f32) / 2f32)
                / crate::HEX_WIDTH;
            // Vertical index
            let j = ((crate::HEX_HEIGHT * self.height as f32 / 2f32)
                - ((i as usize % 2) as f32 * crate::HEX_HEIGHT / 2f32)
                + (y + crate::HEX_HEIGHT / 2f32))
                / crate::HEX_HEIGHT;
            // Since we checked pixel coordinates are within bounds we know index is within range.
            Some([i as usize, j as usize])
        } else {
            None
        }
    }
    /// Returns if a given index is within bounds.
    pub fn contains_index(&self, [x, y]: [usize; 2]) -> bool {
        x < self.width && y < self.height
    }
    /// Render background hexagons on grid.
    pub fn spawn_background(&self, commands: &mut Commands, asset_server: &AssetServer) {
        #[cfg(debug_assertions)]
        println!("spawn_background() started");

        for y in 0..self.height {
            for x in 0..self.width {
                let [cx, cy] = self.logical_pixels([x, y]).unwrap();
                // Spawns hex guide
                // -----------------------------------
                commands.spawn_bundle(GeometryBuilder::build_as(
                    &bevy_prototype_lyon::shapes::RegularPolygon {
                        sides: 6,
                        center: Vec2::new(cx, cy),
                        feature: RegularPolygonFeature::SideLength(crate::HEX_SIDE_LENGTH),
                    },
                    DrawMode::Outlined {
                        fill_mode: FillMode::color(HEX_COLOR),
                        outline_mode: StrokeMode::new(
                            crate::HEX_OUTLINE_COLOR,
                            crate::HEX_OUTLINE_WIDTH,
                        ),
                    },
                    Transform::default(),
                ));
                // // Spawns background texture sprites
                // // -----------------------------------
                // // Elements higher up on the y axis we want to be behind elements lower down, thus we subtract this from the z dimension.
                // let z_adj = (y as f32 / self.logical_pixel_bounds[1].end)
                //     + (x as f32 / self.logical_pixel_bounds[0].end);
                // commands.spawn_bundle(SpriteBundle {
                //     transform: Transform {
                //         translation: Vec3::from((cx, cy, 0f32 - z_adj)),
                //         scale: Vec3::new(0.43f32, 0.43f32, 1f32),
                //         ..Default::default()
                //     },
                //     // sprite: Sprite {
                //     //     color: Color::rgb(1., 1., 1.),
                //     //     ..Default::default()
                //     // },sd
                //     texture: asset_server.load("grass-hex.png"),
                //     ..Default::default()
                // });
            }
        }

        #[cfg(debug_assertions)]
        println!("spawn_background() finished");
    }
    /// Removes highlight for hex highlighted by `COLOUR` returning whether the highlight was present.
    pub fn remove_highlight(&mut self, commands: &mut Commands, colour: Colour) -> bool {
        // If highlight exists, remove it
        match self.highlights.remove(&colour) {
            Some(existing_highlight) => {
                commands.entity(existing_highlight).despawn();
                true
            }
            None => false,
        }
    }
    /// Highlights specific hex with `COLOUR`.
    ///
    /// This function returns:
    /// - `Ok(true)` If the given hex could be highlighted and an existing highlight was present.
    /// - `Ok(false)` If the given hex could be highlighted and an existing highlight wasn't present.
    /// - `Err(str)` If the given hex could not be highlighted (the coordinates where outside the grid).
    pub fn highlight_cell(
        &mut self,
        commands: &mut Commands,
        index: [usize; 2],
        colour: Colour,
    ) -> Result<bool, &str> {
        if self.contains_index(index) {
            // Removes highlight if it exists.
            let removed = self.remove_highlight(commands, colour);

            let hex = self.logical_pixels(index).unwrap();
            let highlight = crate::spawn_hex(hex, *colour, *colour, commands);
            self.highlights.insert(colour, highlight);
            Ok(removed)
        } else {
            Err("Hex coordinates given where outside grid thus hex could not be highlighted.")
        }
    }
    /// Returns a reference to an element returning `None` if the given indices are out of bounds.
    pub fn get(&self, index: [usize; 2]) -> Option<&T> {
        if self.contains_index(index) {
            Some(&self[index])
        } else {
            None
        }
    }
    /// Returns a mutable reference to an element returning `None` if the given indices are out of bounds.
    pub fn get_mut(&mut self, index: [usize; 2]) -> Option<&mut T> {
        if self.contains_index(index) {
            Some(&mut self[index])
        } else {
            None
        }
    }
    const DIR_DIFFS: [[[isize; 2]; 6]; 2] = [
        // even cols
        [[1, 0], [1, -1], [0, -1], [-1, -1], [-1, 0], [0, 1]],
        // odd cols
        [[1, 1], [1, 0], [0, -1], [-1, 0], [-1, 1], [0, 1]],
    ];
    /// Returns coordinates of neighbors, if these coordinates are present in the grid.
    ///
    /// `self.neighbors([2,1])` returns `[Some([3,1]),Some([3,0]),Some([2,0]),Some([1,0]),Some([1,1]),None]`
    /// ```text
    ///       ___     ___     ___     ___
    ///   ___╱1,7╲___╱3,7╲___╱5,7╲___╱7,7╲___
    ///  ╱0,7╲___╱2,7╲___╱4,7╲_2_╱6,7╲___╱8,7╲
    ///  ╲___╱1,6╲___╱3,6╲_2_╱5,6╲_2_╱7,6╲___╱
    ///  ╱0,6╲___╱2,6╲_2_╱4,6╲_1_╱6,6╲_2_╱8,6╲
    ///  ╲___╱1,5╲___╱3,5╲_1_╱5,5╲_1_╱7,5╲___╱
    ///  ╱0,5╲___╱2,5╲_2_╱4,5╲_0_╱6,5╲_2_╱8,5╲
    ///  ╲___╱1,4╲___╱3,4╲_1_╱5,4╲_1_╱7,4╲___╱
    ///  ╱0,4╲___╱2,4╲_2_╱4,4╲_1_╱6,4╲_2_╱8,4╲
    ///  ╲___╱1,3╲___╱3,3╲_2_╱5,3╲_2_╱7,3╲___╱
    ///  ╱0,3╲___╱2,3╲___╱4,3╲_2_╱6,3╲___╱8,3╲
    ///  ╲___╱1,2╲___╱3,2╲___╱5,2╲___╱7,2╲___╱
    ///  ╱0,2╲___╱2,2╲___╱4,2╲___╱6,2╲___╱8,2╲
    ///  ╲___╱1,1╲___╱3,1╲___╱5,1╲___╱7,1╲___╱
    ///  ╱0,1╲___╱2,1╲___╱4,1╲___╱6,1╲___╱8,1╲
    ///  ╲___╱1,0╲___╱3,0╲___╱5,0╲___╱7,0╲___╱
    ///  ╱0,0╲___╱2,0╲___╱4,0╲___╱6,0╲___╱8,0╲
    ///  ╲___╱   ╲___╱   ╲___╱   ╲___╱   ╲___╱
    /// ```
    /// ```text
    ///  ___     ___
    /// ╱0,0╲___╱2,0╲___
    /// ╲___╱1,0╲___╱3,0╲
    /// ╱0,1╲___╱2,1╲___╱
    /// ╲___╱1,1╲___╱3,1╲
    ///     ╲___╱   ╲___╱
    /// ```
    pub fn neighbors(&self, [col, row]: [usize; 2]) -> [Option<[usize; 2]>; 6] {
        let [col, row] = [col as isize, row as isize];
        let parity = col & 1;
        let group = &Self::DIR_DIFFS[parity as usize];
        [
            self.offset_coord_wrapper(col + group[0][0], row + group[0][1]),
            self.offset_coord_wrapper(col + group[1][0], row + group[1][1]),
            self.offset_coord_wrapper(col + group[2][0], row + group[2][1]),
            self.offset_coord_wrapper(col + group[3][0], row + group[3][1]),
            self.offset_coord_wrapper(col + group[4][0], row + group[4][1]),
            self.offset_coord_wrapper(col + group[5][0], row + group[5][1]),
        ]
    }
    pub fn offset_coord_wrapper(&self, a: isize, b: isize) -> Option<[usize; 2]> {
        match (usize::try_from(a), usize::try_from(b)) {
            (Ok(x), Ok(y)) if self.contains_index([x, y]) => Some([x, y]),
            _ => None,
        }
    }
    pub fn neighbor(&self, [col, row]: [usize; 2], direction: usize) -> Option<[usize; 2]> {
        let [col, row] = [col as isize, row as isize];
        let parity = col & 1;
        let group = &Self::DIR_DIFFS[parity as usize];
        self.offset_coord_wrapper(col + group[direction][0], row + group[direction][1])
    }
}

impl<T: std::fmt::Debug> std::ops::Index<[usize; 2]> for HexGrid<T> {
    type Output = T;
    /// In a `2x4` hex grid the corresponding indices would be:
    /// ```text
    ///  ___     ___
    /// ╱0,0╲___╱2,0╲___
    /// ╲___╱1,0╲___╱3,0╲
    /// ╱0,1╲___╱2,1╲___╱
    /// ╲___╱1,1╲___╱3,1╲
    ///     ╲___╱   ╲___╱
    /// ```
    fn index(&self, [x, y]: [usize; 2]) -> &Self::Output {
        &self.data[y * self.width + x]
    }
}
impl<T: std::fmt::Debug> std::ops::IndexMut<[usize; 2]> for HexGrid<T> {
    /// In a `2x4` hex grid the corresponding indices would be:
    /// ```text
    ///  ___     ___
    /// ╱0,0╲___╱2,0╲___
    /// ╲___╱1,0╲___╱3,0╲
    /// ╱0,1╲___╱2,1╲___╱
    /// ╲___╱1,1╲___╱3,1╲
    ///     ╲___╱   ╲___╱
    /// ```
    fn index_mut(&mut self, [x, y]: [usize; 2]) -> &mut Self::Output {
        &mut self.data[y * self.width + x]
    }
}
/// A wrapper around [`Color`](https://docs.rs/bevy/latest/bevy/render/color/enum.Color.html) to implement [`std::hash::Hash`].
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Colour(pub Color);
/// While this is not technically true, this is necessary to hash it and be useful.
impl Eq for Colour {}
impl std::hash::Hash for Colour {
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        icolour(self.0.r()).hash(state);
        icolour(self.0.g()).hash(state);
        icolour(self.0.b()).hash(state);
        icolour(self.0.a()).hash(state);
    }
}
impl From<[f32; 4]> for Colour {
    fn from(x: [f32; 4]) -> Self {
        Self(Color::from(x))
    }
}
impl std::ops::Deref for Colour {
    type Target = Color;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

/// Converts a value in 0..1 to a value in 0..256
const fn icolour(x: f32) -> Result<u8, &'static str> {
    // match (0f32..1f32).contains(&x)
    // Bc below `std::ops::Range::contains` isn't const.
    match x > 0f32 && x < 1f32 {
        true => Ok((x * 255f32) as u8),
        false => Err("Float outside range 0..1"),
    }
}

/// A description of the map.
#[derive(serde::Deserialize, serde::Serialize)]
pub struct MapDescriptor {
    pub obstacles: Vec<[usize; 2]>,
    pub player_spawns: Vec<[usize; 2]>,
    pub enemy_spawns: Vec<[usize; 2]>,
}

/// The data we store associated with each tile of the hex grid.
#[derive(Debug, Copy, Clone)]
pub enum HexItem {
    Entity(Entity),
    Enemy(Entity),
    Empty,
    Obstruction,
}
impl HexItem {
    pub fn entity(self) -> Entity {
        match self {
            Self::Entity(entity) => entity,
            _ => panic!("called HexItem::entity() on non-entity item"),
        }
    }
    pub fn is_empty(&self) -> bool {
        matches!(self, Self::Empty)
    }
}
impl Default for HexItem {
    fn default() -> Self {
        Self::Empty
    }
}

/// The list of all [`Laser`]s present on screen.
#[derive(Debug, Default)]
pub struct Lasers(pub Vec<Laser>);

/// Data associated with a laser projectile after being fired by a unit.
#[derive(Debug, Clone)]
pub struct Laser {
    pub entity: Entity,
    pub opacity: f32,
    // Opacity decay every tick
    pub decay: f32,
    pub timer: Timer,
    pub from: Vec2,
    pub to: Vec2,
}
impl std::ops::Deref for Lasers {
    type Target = Vec<Laser>;
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
impl std::ops::DerefMut for Lasers {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

/// The entities associated with cursor aiming a unit.
#[derive(Default, Debug)]
pub struct FiringLines(pub Vec<Entity>);

/// The entities associated with displaying the tiles a projection crossed post a unit firing a
///  weapon (for the moment this is only fof debug mode).
#[derive(Default, Debug)]
pub struct FiringPath(pub Vec<Entity>);

/// The current user selected unit.
#[derive(Default, Debug)]
pub struct SelectedUnitOption(pub Option<[usize; 2]>);

#[derive(Component, Debug)]
pub struct EnemyUnit(pub Unit);
impl std::ops::Deref for EnemyUnit {
    type Target = Unit;
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
impl std::ops::DerefMut for EnemyUnit {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

static UnitCounter: AtomicUsize = AtomicUsize::new(0);

#[derive(Component, Debug)]
pub struct FiringSpread();

pub const FIRING_SPREAD_WIDTH: u32 = 1000;
pub const FIRING_SPREAD_HEIGHT: u32 = 1000;
/// A player or AI controlled unit/soldier/pawn.
#[derive(Component, Debug)]
pub struct Unit {
    // The angles of lines splitting the firing arc into `DISTRIBUTION_VISIBLE_SECTIONS` slice
    //  where each slice has an equal percentage of a shot landing within it.
    pub firing_buckets: [f32; crate::DISTRIBUTION_BUCKETS],
    // Distribution from which we sample our firing inaccuracies.
    pub firing_distribution: rand_distr::Normal<f32>,
    // Time unit takes to move 1 hex.
    pub movement_time: f32,
    // Time unit takes to fire once.
    pub firing_time: f32,
    // Remaining time this unit has this turn.
    pub remaining_time: f32,
    // Time unit has each turn.
    pub time: f32,
    // Data associated with unit movement
    pub movement_data: MovementData,
    pub accuracy_field: Entity,
}
impl Unit {
    pub fn show_time(
        &mut self,
        hex_grid: &HexGrid<HexItem>,
        commands: &mut Commands,
        asset_server: &AssetServer,
    ) {
        if self.movement_data.remaining_time.is_empty() {
            for (index, distance) in self.movement_data.reachable_tiles.iter() {
                let [x, y] = hex_grid.logical_pixels(*index).unwrap();
                let time_remaining_after_move =
                    self.remaining_time - *distance as f32 * self.movement_time;
                self.movement_data.remaining_time.push(crate::spawn_text(
                    (time_remaining_after_move as u32).to_string(),
                    [
                        x,
                        y - crate::MOVEMENT_TIME_REMAINING_FONT_SIZE / 2f32
                            - crate::REACHABLE_TEXT_SPACING,
                    ],
                    crate::MOVEMENT_TIME_REMAINING_FONT_SIZE,
                    commands,
                    asset_server,
                ));
            }
        }
    }
    fn default(commands: &mut Commands, asset_server: &AssetServer, [x, y]: [f32; 2]) -> Entity {
        const DEFAULT_UNIT_ACCURACY_DEVIATION: f32 = 0.5;
        let firing_distribution =
            rand_distr::Normal::new(0f32, DEFAULT_UNIT_ACCURACY_DEVIATION).unwrap();
        let buckets = crate::buckets(firing_distribution);
        // println!("buckets: {:.3?}", buckets);

        // Since we want to layer the probabilities on top of each other
        let mut cumulative_buckets = [0f32; crate::DISTRIBUTION_BUCKETS];
        for i in 0..crate::DISTRIBUTION_BUCKETS {
            for j in i..crate::DISTRIBUTION_BUCKETS {
                cumulative_buckets[i] += buckets[j];
            }
        }
        // println!("cumulative_buckets: {:.3?}",cumulative_buckets);

        // Calculating
        // -----------------------------------------------------------------------------------------

        const START: [f32; 2] = [0f32, FIRING_SPREAD_HEIGHT as f32 / 2f32];
        const END: [f32; 2] = [
            FIRING_SPREAD_WIDTH as f32,
            FIRING_SPREAD_WIDTH as f32 / 2f32,
        ];
        // Forms image/texture we use for accuracy spread to avoid recalculation.
        let mut spread_image = image::RgbaImage::new(FIRING_SPREAD_WIDTH, FIRING_SPREAD_HEIGHT);
        // Draw slices
        let mut angle_offset = crate::DISTRIBUTION_SAMPLES_STEP;
        let start = imageproc::point::Point {
            x: START[0] as i32,
            y: START[1] as i32,
        };
        let end = imageproc::point::Point {
            x: END[0] as i32,
            y: END[1] as i32,
        };
        let (mut neg_last, mut pos_last) = (end.clone(), end.clone());

        #[cfg(debug_assertions)]
        println!("Drawing");

        for i in 0..crate::DISTRIBUTION_BUCKETS {
            let colour = {
                let c = crate::get_gradient_colour(cumulative_buckets[i]);
                image::Rgba([
                    (c[0] * 255f32) as u8,
                    (c[1] * 255f32) as u8,
                    (c[2] * 255f32) as u8,
                    (c[3] * 255f32) as u8,
                ])
            };
            let neg_vec = {
                let nv =
                    crate::end_point_unbound(START, END, -angle_offset, FIRING_SPREAD_WIDTH as f32);
                // println!("nv: {:?}",nv);
                imageproc::point::Point {
                    x: nv[0] as i32,
                    y: nv[1] as i32,
                }
            };
            let pos_vec = {
                let pv =
                    crate::end_point_unbound(START, END, angle_offset, FIRING_SPREAD_WIDTH as f32);
                imageproc::point::Point {
                    x: pv[0] as i32,
                    y: pv[1] as i32,
                }
            };
            // println!("{:?} -> {:?} -> {:?}",neg_last,neg_vec,start);
            imageproc::drawing::draw_polygon_mut(
                &mut spread_image,
                &[neg_last, neg_vec, start],
                colour,
            );
            imageproc::drawing::draw_polygon_mut(
                &mut spread_image,
                &[pos_last, pos_vec, start],
                colour,
            );
            neg_last = neg_vec;
            pos_last = pos_vec;
            angle_offset += crate::DISTRIBUTION_SAMPLES_STEP;
        }

        // Saving
        // -----------------------------------------------------------------------------------------

        #[cfg(debug_assertions)]
        let start_inst = std::time::Instant::now();

        let count = UnitCounter.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
        std::fs::create_dir_all("./assets/cache").unwrap();
        spread_image
            .save(format!("./assets/cache/{}.png", count))
            .unwrap();

        #[cfg(debug_assertions)]
        println!("Saved in {:.3?}", start_inst.elapsed());

        // Creating
        // -----------------------------------------------------------------------------------------

        // Scale the accuracy field to the screen resolution
        let accuracy_field = commands
            .spawn_bundle(SpriteBundle {
                transform: Transform {
                    translation: Vec3::from((x + FIRING_SPREAD_WIDTH as f32 / 2f32, y, -1f32)),
                    scale: Vec3::new(1f32, 1f32, 1f32),
                    ..Default::default()
                },
                texture: asset_server.load(&format!("cache/{}.png", count)),
                ..Default::default()
            })
            .insert(FiringSpread())
            .id();
        let new_entity = commands
            .spawn_bundle(SpriteBundle {
                transform: Transform {
                    translation: Vec3::from((x, y, 1f32)),
                    scale: Vec3::new(1f32, 1f32, 1f32),
                    ..Default::default()
                },
                texture: asset_server.load(UNIT_SPRITE_SINGLE),
                ..Default::default()
            })
            .insert(Self {
                firing_buckets: cumulative_buckets,
                firing_distribution,
                movement_time: 10f32,
                firing_time: 30f32,
                remaining_time: 100f32,
                time: 100f32,
                movement_data: Default::default(),
                accuracy_field,
            })
            .insert(Animation::from_dir(UNIT_SPRITE_PATH, asset_server).unwrap())
            .id();

        #[cfg(debug_assertions)]
        println!("Drawn in {:.3?}", start_inst.elapsed());

        new_entity
    }
}

/// Data associated with movement of a unit.
#[derive(Debug, Default)]
pub struct MovementData {
    // Distances to all reachable points from current position.
    pub reachable_tiles: HashMap<[usize; 2], usize>,
    // Entities highlighting reachable tiles.
    pub tile_highlights: Vec<Entity>,
    // Entities of text denoting remaining time on reachable tiles after moving to them.
    pub remaining_time: Vec<Entity>,
    // Entities of text denoting possible shots from reachable tiles after moving to them.
    pub possible_shots: Vec<Entity>,
}
impl MovementData {
    pub fn clear(&mut self, commands: &mut Commands) {
        for tile_highlight in self.tile_highlights.iter() {
            commands.entity(*tile_highlight).despawn();
        }
        for remaining_time in self.remaining_time.iter() {
            commands.entity(*remaining_time).despawn();
        }
        for possible_shot in self.possible_shots.iter() {
            commands.entity(*possible_shot).despawn();
        }
        *self = Default::default();
    }
    pub fn clear_time(&mut self, commands: &mut Commands) {
        for remaining_time in self.remaining_time.iter() {
            commands.entity(*remaining_time).despawn();
        }
        self.remaining_time = Vec::new();
    }
}
