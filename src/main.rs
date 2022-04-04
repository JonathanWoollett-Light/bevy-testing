#![allow(incomplete_features)]
#![feature(adt_const_params)]
#![feature(const_fn_floating_point_arithmetic)]
use bevy::{input::mouse::MouseWheel, prelude::*};
use bevy_kira_audio::{Audio, AudioPlugin};
use bevy_prototype_lyon::prelude::*;
use rand_distr::Distribution;
use std::collections::{HashMap, HashSet};

/// Colour of lasers shot by units.
const LASER_COLOUR: Color = Color::rgba(1f32, 0f32, 0f32, 1f32);
/// Background colour.
const BACKGROUND_COLOUR: Color = Color::rgb(0f32, 0f32, 0f32);
/// Colour highlight on selected units.
const UNIT_SELECTION_COLOUR: Colour = Colour(Color::rgba(0f32, 0f32, 1f32, 0.2f32));
/// Minimum camera zoom/scale.
const MIN_ZOOM: f32 = 1f32;
/// Maximum camera zoom/scale.
const MAX_ZOOM: f32 = 5f32;
/// Colour highlight for hex's the user's cursor is hovering over.
const HOVER_COLOUR: Colour = Colour(Color::rgba(1f32, 1f32, 1f32, 0.5f32));
/// The square root of 3 `3^0.5`
///
/// Since [`f32::sqrt()`] is not a const fn.
const SQRT_3: f32 = 1.7320508f32;
/// Length of 1 side of a hexagon for the hexagons used to form the map.
const HEX_SIDE_LENGTH: f32 = 30f32;
/// Width of outline used for the hexagons used to form the map.
const HEX_OUTLINE_WIDTH: f32 = 3f32;
/// Width of hexagons used to form the map.
const HEX_WIDTH: f32 = 3f32 * HEX_SIDE_LENGTH / 2f32;
/// Height of hexagons used to form the map.
const HEX_HEIGHT: f32 = (SQRT_3 / 2f32) * (HEX_SIDE_LENGTH / 0.5f32);
/// Colour of hexagons used to form the map.
const HEX_COLOR: Color = Color::rgba(1f32, 1f32, 1f32, 0.6f32);
/// Colour of outline of hexagons used to form the map.
const HEX_OUTLINE_COLOR: Color = Color::rgba(0f32, 0f32, 0f32, 0.2f32);
/// Samples to take from firing distributions to approximate firing accuracies.
const SAMPLES: usize = 10;
/// Fill colour of obstructed tiles on map.
const OBSTRUCTION_HEX_COLOUR: Color = Color::rgba(1f32, 1f32, 1f32, 1f32);
/// Outline colour of obstructed tiles on the map.
const OBSTACLE_HEX_OUTLINE_COLOUR: Color = Color::rgba(0f32, 0f32, 0f32, 0.8f32);
/// Firing lines colour.
const FIRING_PATH_COLOUR: Color = Color::rgba(1., 1., 1., 0.2);
/// Z position of firing lines.
const FIRING_LINE_Z: f32 = 5f32;
/// Width of laser rectangles.
const LASER_WIDTH: f32 = 3f32;

/// Converts a value in 0..1 to a value in 0..256
const fn icolour(x: f32) -> Result<u8, &'static str> {
    // match (0f32..1f32).contains(&x)
    // Bc below `std::ops::Range::contains` isn't const.
    match x > 0f32 && x < 1f32 {
        true => Ok((x * 255f32) as u8),
        false => Err("Float outside range 0..1"),
    }
}

/// An hexagon grid.
#[derive(Debug, Clone)]
struct HexGrid<T: std::fmt::Debug> {
    data: Vec<T>,
    pub width: usize,
    pub height: usize,
    /// Logical pixel bounds of hex grid.
    pub logical_pixel_bounds: [std::ops::Range<f32>; 2],
    /// A mapping of colour highlights to their respective entity.
    highlights: HashMap<Colour, Entity>,
}

use rand::{thread_rng, Rng};

#[derive(Debug, Component)]
struct Animation {
    timer: Timer,
    frames: std::iter::Cycle<std::vec::IntoIter<bevy::prelude::Handle<Image>>>,
}
impl Animation {
    fn from_dir(path: &str, asset_server: &AssetServer) -> std::io::Result<Self> {
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
    fn tick(&mut self, step: bevy::utils::Duration) -> Option<Handle<Image>> {
        if self.timer.tick(step).just_finished() {
            self.frames.next()
        } else {
            None
        }
    }
}
const UNIT_SPRITE_SINGLE: &'static str = "units/enemy1idle1.png";
const UNIT_SPRITE_PATH: &'static str = "./assets/units";
const ENEMY_SPRITE_SINGLE: &'static str = "enemy/enemy3idle1.png";
const ENEMY_SPRITE_PATH: &'static str = "./assets/enemy";
impl HexGrid<HexItem> {
    fn add_enemy(
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

    fn add_unit(
        &mut self,
        commands: &mut Commands,
        component: Unit,
        index: [usize; 2],
        asset_server: &AssetServer,
    ) {
        #[cfg(debug_assertions)]
        println!("add_unit() started");
        let [x, y] = self.logical_pixels(index).unwrap();
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
            .insert(component)
            .insert(Animation::from_dir(UNIT_SPRITE_PATH, asset_server).unwrap())
            .id();
        self[index] = HexItem::Entity(new_entity);

        #[cfg(debug_assertions)]
        println!("add_unit() finished");
    }
    fn add_obstruction(
        &mut self,
        commands: &mut Commands,
        index: [usize; 2],
        asset_server: &AssetServer,
    ) {
        let [x, y] = self.logical_pixels(index).unwrap();
        // Elements higher up on the y axis we want to be behind elements lower down, thus we subtract this from the z dimension.
        let z_adj = y as f32 / self.logical_pixel_bounds[1].end;
        println!("y: {:?}, z_adj: {}", y, z_adj);
        spawn_hex(
            [x,y],
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
    fn reachable(&self, start: [usize; 2], movement: usize) -> Vec<Vec<[usize; 2]>> {
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
    fn new(width: usize, height: usize) -> Self {
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
    fn logical_pixels(&self, index: [usize; 2]) -> Option<[f32; 2]> {
        if self.contains_index(index) {
            Some(self.unchecked_logical_pixels(index))
        } else {
            None
        }
    }
    /// Gets logical pixel coordinates of center of hex of a given index.
    fn unchecked_logical_pixels(&self, [x, y]: [usize; 2]) -> [f32; 2] {
        [
            -(HEX_WIDTH * self.width as f32 / 2f32) + HEX_WIDTH * x as f32,
            -(HEX_HEIGHT * self.height as f32 / 2f32)
                + (HEX_HEIGHT * y as f32)
                + ((x % 2) as f32 * HEX_HEIGHT / 2f32),
        ]
    }
    /// Gets index of logical pixel coordinates in hex grid, if the the given coordinates are within the grid.
    fn index(&self, [x, y]: [f32; 2]) -> Option<[usize; 2]> {
        if self.logical_pixel_bounds[0].contains(&x) && self.logical_pixel_bounds[1].contains(&y) {
            // Horizontal index
            let i = ((x + HEX_WIDTH / 2f32) + (HEX_WIDTH * self.width as f32) / 2f32) / HEX_WIDTH;
            // Vertical index
            let j = ((HEX_HEIGHT * self.height as f32 / 2f32)
                - ((i as usize % 2) as f32 * HEX_HEIGHT / 2f32)
                + (y + HEX_HEIGHT / 2f32))
                / HEX_HEIGHT;
            // Since we checked pixel coordinates are within bounds we know index is within range.
            Some([i as usize, j as usize])
        } else {
            None
        }
    }
    /// Returns if a given index is within bounds.
    fn contains_index(&self, [x, y]: [usize; 2]) -> bool {
        x < self.width && y < self.height
    }
    /// Render background hexagons on grid.
    fn spawn_background(&self, commands: &mut Commands, asset_server: &AssetServer) {
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
                        feature: RegularPolygonFeature::SideLength(HEX_SIDE_LENGTH),
                    },
                    DrawMode::Outlined {
                        fill_mode: FillMode::color(HEX_COLOR),
                        outline_mode: StrokeMode::new(HEX_OUTLINE_COLOR, HEX_OUTLINE_WIDTH),
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
    fn remove_highlight(&mut self, commands: &mut Commands, colour: Colour) -> bool {
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
    fn highlight_cell(
        &mut self,
        commands: &mut Commands,
        index: [usize; 2],
        colour: Colour,
    ) -> Result<bool, &str> {
        if self.contains_index(index) {
            // Removes highlight if it exists.
            let removed = self.remove_highlight(commands, colour);

            let hex = self.logical_pixels(index).unwrap();
            let highlight = spawn_hex(hex, *colour, *colour, commands);
            self.highlights.insert(colour, highlight);
            Ok(removed)
        } else {
            Err("Hex coordinates given where outside grid thus hex could not be highlighted.")
        }
    }
    /// Returns a reference to an element returning `None` if the given indices are out of bounds.
    fn get(&self, index: [usize; 2]) -> Option<&T> {
        if self.contains_index(index) {
            Some(&self[index])
        } else {
            None
        }
    }
    /// Returns a mutable reference to an element returning `None` if the given indices are out of bounds.
    fn get_mut(&mut self, index: [usize; 2]) -> Option<&mut T> {
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
    fn neighbors(&self, [col, row]: [usize; 2]) -> [Option<[usize; 2]>; 6] {
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
    fn offset_coord_wrapper(&self, a: isize, b: isize) -> Option<[usize; 2]> {
        match (usize::try_from(a), usize::try_from(b)) {
            (Ok(x), Ok(y)) if self.contains_index([x, y]) => Some([x, y]),
            _ => None,
        }
    }
    fn neighbor(&self, [col, row]: [usize; 2], direction: usize) -> Option<[usize; 2]> {
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
struct Colour(Color);
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

/// A description of the map.
#[derive(serde::Deserialize, serde::Serialize)]
struct MapDescriptor {
    obstacles: Vec<[usize; 2]>,
    player_spawns: Vec<[usize; 2]>,
    enemy_spawns: Vec<[usize; 2]>,
}

/// The data we store associated with each tile of the hex grid.
#[derive(Debug, Copy, Clone)]
enum HexItem {
    Entity(Entity),
    Enemy(Entity),
    Empty,
    Obstruction,
}
impl HexItem {
    fn entity(self) -> Entity {
        match self {
            Self::Entity(entity) => entity,
            _ => panic!("called HexItem::entity() on non-entity item"),
        }
    }
    fn is_empty(&self) -> bool {
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
struct Lasers(Vec<Laser>);

/// Data associated with a laser projectile after being fired by a unit.
#[derive(Debug, Clone)]
struct Laser {
    entity: Entity,
    opacity: f32,
    // Opacity decay every tick
    decay: f32,
    timer: Timer,
    from: Vec2,
    to: Vec2,
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
struct FiringLines(Vec<Entity>);

/// The entities associated with displaying the tiles a projection crossed post a unit firing a 
///  weapon (for the moment this is only fof debug mode).
#[derive(Default, Debug)]
struct FiringPath(Vec<Entity>);

/// The current user selected unit.
#[derive(Default, Debug)]
struct SelectedUnitOption(Option<[usize; 2]>);

#[derive(Component, Debug, Default)]
struct EnemyUnit(Unit);
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


// With:
// - 0 we draw no slices in the firing arc, 
// - 1 we draw 2 lines which split the firing arc into 2 sections each with 50% chance of a shot lying within them.
// - 2 we draw 4 lines splitting the firing arc into 4 sections each with a 25% chance of a shot lying within them.
const DISTRIBUTION_VISIBLE_SECTIONS: usize = 2;
const VISIBLE_FIRING_STEP: f32 = 1f32 / (DISTRIBUTION_VISIBLE_SECTIONS as f32 + f32::EPSILON);

/// A player or AI controlled unit/soldier/pawn.
#[derive(Component, Debug)]
struct Unit {
    // The angles of lines splitting the firing arc into `DISTRIBUTION_VISIBLE_SECTIONS` slice 
    //  where each slice has an equal percentage of a shot landing within it.
    firing_buckets: [f32; DISTRIBUTION_VISIBLE_SECTIONS],
    // Distribution from which we sample our firing inaccuracies.
    firing_distribution: rand_distr::Normal<f32>,
    // Time unit takes to move 1 hex.
    movement_time: f32,
    // Time unit takes to fire once.
    firing_time: f32,
    // Remaining time this unit has this turn.
    remaining_time: f32,
    // Time unit has each turn.
    time: f32,
    // Data associated with unit movement
    movement_data: MovementData,
}
impl Unit {
    fn show_time(
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
                self.movement_data.remaining_time.push(spawn_text(
                    (time_remaining_after_move as u32).to_string(),
                    [
                        x,
                        y - MOVEMENT_TIME_REMAINING_FONT_SIZE / 2f32 - REACHABLE_TEXT_SPACING,
                    ],
                    MOVEMENT_TIME_REMAINING_FONT_SIZE,
                    commands,
                    asset_server,
                ));
            }
        }
    }
}
impl Default for Unit {
    fn default() -> Self {
        let firing_distribution = rand_distr::Normal::new(0f32, 0.05f32).unwrap();
        let buckets = buckets(firing_distribution);
        println!("buckets: {:.3?}",buckets);

        let mut sections = [0f32; DISTRIBUTION_VISIBLE_SECTIONS];
        let mut j = 0;
        for i in 0..DISTRIBUTION_VISIBLE_SECTIONS {
            let mut percent = 0f32;
            while percent < VISIBLE_FIRING_STEP && j < DISTRIBUTION_BUCKETS {
                percent += buckets[j];
                sections[i] += DISTRIBUTION_SAMPLES_STEP;
                j += 1;
            }
        }

        Self {
            firing_buckets: sections,
            firing_distribution,
            movement_time: 10f32,
            firing_time: 30f32,
            remaining_time: 100f32,
            time: 100f32,
            movement_data: Default::default(),
        }
    }
}

/// Data associated with movement of a unit.
#[derive(Debug, Default)]
struct MovementData {
    // Distances to all reachable points from current position.
    reachable_tiles: HashMap<[usize; 2], usize>,
    // Entities highlighting reachable tiles.
    tile_highlights: Vec<Entity>,
    // Entities of text denoting remaining time on reachable tiles after moving to them.
    remaining_time: Vec<Entity>,
    // Entities of text denoting possible shots from reachable tiles after moving to them.
    possible_shots: Vec<Entity>,
}
impl MovementData {
    fn clear(&mut self, commands: &mut Commands) {
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
    fn clear_time(&mut self, commands: &mut Commands) {
        for remaining_time in self.remaining_time.iter() {
            commands.entity(*remaining_time).despawn();
        }
        self.remaining_time = Vec::new();
    }
}

fn main() {
    App::new()
        .insert_resource(ClearColor(BACKGROUND_COLOUR))
        .add_plugins(DefaultPlugins)
        .add_plugin(ShapePlugin)
        .add_plugin(AudioPlugin)
        .add_startup_system(setup)
        .add_system(unit_movement_system)
        .add_system(camera_movement_system)
        .add_system(hover_system)
        .add_system(turnover_system)
        .add_system(firing_system::<0.05f32, 0.05f32>)
        .add_system(animation_system)
        .run();
}

/// Sets up initial system state.
fn setup(mut commands: Commands, asset_server: Res<AssetServer>, audio: Res<Audio>) {
    let asset_server = asset_server.into_inner();
    #[cfg(debug_assertions)]
    println!("setup() started");
    // cameras
    commands.spawn_bundle(OrthographicCameraBundle::new_2d());
    commands.spawn_bundle(UiCameraBundle::default());
    // Shoots remain on screen for 0.5 seconds
    commands.insert_resource(Lasers::default());

    // Background
    let grid_height = 20;
    let grid_width = 39;

    let mut hex_grid = HexGrid::new(grid_width, grid_height);
    hex_grid.spawn_background(&mut commands, asset_server);

    // Adds some obstructions
    let map: MapDescriptor =
        serde_json::from_str(&std::fs::read_to_string("map.json").unwrap()).unwrap();

    for hex in map.obstacles.into_iter() {
        hex_grid.add_obstruction(&mut commands, hex, asset_server);
    }
    #[cfg(debug_assertions)]
    println!("spawned obstructions");

    let mut units = Vec::new();
    for hex in map.player_spawns {
        println!("hex: {:?}", hex);
        units.push((Unit::default(), hex));
    }
    // let units = map
    //     .player_spawns
    //     .into_iter()
    //     .map(|hex| (Unit::default(), hex))
    //     .collect::<Vec<_>>();

    let mut enemy_units = Vec::new();
    for hex in map.enemy_spawns {
        println!("hex: {:?}", hex);
        enemy_units.push((EnemyUnit::default(), hex));
    }

    #[cfg(debug_assertions)]
    println!("units: {:?}", units);

    // Add units
    // ----------------------------------------------------

    #[cfg(debug_assertions)]
    println!("started spawning units");

    for (unit, index) in units.into_iter() {
        hex_grid.add_unit(&mut commands, unit, index, asset_server);
    }
    #[cfg(debug_assertions)]
    println!("spawned units");

    #[cfg(debug_assertions)]
    println!("started spawning enemies");
    for (enemy, index) in enemy_units.into_iter() {
        hex_grid.add_enemy(&mut commands, enemy, index, asset_server);
    }
    #[cfg(debug_assertions)]
    println!("spawned enemies");

    commands.insert_resource(hex_grid);
    commands.insert_resource(SelectedUnitOption::default());
    commands.insert_resource(FiringLines::default());
    #[cfg(debug_assertions)]
    commands.insert_resource(FiringPath::default());

    // Play background music
    let channel = bevy_kira_audio::AudioChannel::new(String::from("background-music"));
    audio.set_volume_in_channel(0.1, &channel);
    audio.play_looped_in_channel(
        asset_server
            .load("XCOM-Enemy-Unknown-Soundtrack-HQ-Act-1-Extended-Michael-McCann-YouTube.mp3"),
        &channel,
    );

    #[cfg(debug_assertions)]
    println!("setup() finished");
}

/// Handles highlighting the tile the user's cursor is hovering over.
fn hover_system(
    windows: Res<Windows>,
    mut commands: Commands,
    mut cursor_events: EventReader<CursorMoved>,
    hex_grid: ResMut<HexGrid<HexItem>>,
    camera_query: Query<(&Transform, With<bevy::prelude::Camera>)>,
) {
    let hex_grid = hex_grid.into_inner();
    // Since we want the highlighted hex to update on camera movement we don't use the cursor event.
    for cursor_event in cursor_events.iter() {
        // #[cfg(debug_assertions)]
        // println!("cursor event received");

        let window = windows.get_primary().expect("no primary window");
        let cursor_position1 =
            cursor_event.position - Vec2::new(window.width() / 2., window.height() / 2.);

        // TODO Here we skip our 1st camera (the ui camera) do this better
        let (camera_transform, _) = camera_query.iter().nth(1).unwrap();
        let cursor_position2 = normalize_cursor_position(cursor_position1, camera_transform);

        // #[cfg(debug_assertions)]
        // println!(
        //     "received cursor_positions: {:.0}->{:.0}->{:.0}",
        //     cursor_event.position, cursor_position1, cursor_position2
        // );

        let index = hex_grid.index(cursor_position2.to_array());

        // #[cfg(debug_assertions)]
        // println!("index: {:?}", index);

        match index {
            // If both logical pixel coordinates can be mapped to hexes within our hex grid
            Some([x, y]) => {
                hex_grid
                    .highlight_cell(&mut commands, [x, y], HOVER_COLOUR)
                    .unwrap();
            }
            // If either logical pixel coordinates are outside our hex grid
            _ => {
                hex_grid.remove_highlight(&mut commands, HOVER_COLOUR);
            }
        }
    }
}

/// Scales and translates cursor position relative to camera to get a global cursor position.
fn normalize_cursor_position(pos: Vec2, camera_transform: &Transform) -> Vec2 {
    (pos * camera_transform.scale.truncate()) + camera_transform.translation.truncate()
}

/// Handles game turn over e.g. clicking "next turn".
fn turnover_system(
    keys: Res<Input<KeyCode>>,
    mut query: Query<&mut Unit>,
    selected_entity: ResMut<SelectedUnitOption>,
    hex_grid: ResMut<HexGrid<HexItem>>,
    mut commands: Commands,
    asset_server: Res<AssetServer>,
    camera_query: Query<(&Transform, With<bevy::prelude::Camera>)>,
) {
    // TODO Here we effectively skip ui camera, do this better
    let (camera_transform, _) = camera_query.iter().nth(1).unwrap();
    if keys.just_pressed(KeyCode::Space) {
        query.for_each_mut(|mut f| {
            f.remaining_time = f.time;
        });
        // Since we may be changing the movement of a selected unit we need to update the movement range of this unit
        if let Some(selected) = selected_entity.0 {
            let selected_unit = query
                .get_mut(hex_grid[selected].entity())
                .unwrap()
                .into_inner();

            // Update movement range
            clear_reachable(selected_unit, &mut commands);
            render_reachable(
                selected_unit,
                &mut commands,
                selected,
                &hex_grid,
                &asset_server,
                *camera_transform,
            );
        }
    }
}

/// System handling unit movement
fn unit_movement_system(
    selected_entity: ResMut<SelectedUnitOption>,
    mut commands: Commands,
    buttons: Res<Input<MouseButton>>,
    mut query: Query<(Entity, &mut Unit, &mut Transform)>,
    windows: Res<Windows>,
    hex_grid: ResMut<HexGrid<HexItem>>,
    camera_query: Query<(&Transform, With<bevy::prelude::Camera>, Without<Unit>)>,
    asset_server: Res<AssetServer>,
    firing_line: ResMut<FiringLines>,
    audio: Res<Audio>,
) {
    let window = windows.get_primary().expect("no primary window");
    // Right click de-selects unit if a unit is selected
    let selected_entity = selected_entity.into_inner();
    let hex_grid = hex_grid.into_inner();
    let firing_line = firing_line.into_inner();
    if buttons.just_pressed(MouseButton::Right) {
        // #[cfg(debug_assertions)]
        // println!("right click");
        if let Some(selected_unit) = selected_entity.0 {
            let (_, unit, _) = query.get_mut(hex_grid[selected_unit].entity()).unwrap();
            let unit = unit.into_inner();
            // De-spawns old movement components
            clear_reachable(unit, &mut commands);
        }
        // Remove selected unit
        selected_entity.0 = None;
        // Remove unit position highlight
        hex_grid.remove_highlight(&mut commands, UNIT_SELECTION_COLOUR);

        // Removes old firing lines
        for line in firing_line.0.iter() {
            commands.entity(*line).despawn();
        }
        firing_line.0 = Vec::new();
    }
    if buttons.just_pressed(MouseButton::Left) {
        // #[cfg(debug_assertions)]
        // println!("left click");

        let cursor_position = window.cursor_position().unwrap()
            - Vec2::new(window.width() / 2., window.height() / 2.);
        // TODO Here we effectively skip ui camera, do this better
        let (camera_transform, _, _) = camera_query.iter().nth(1).unwrap();
        let cursor_position = normalize_cursor_position(cursor_position, camera_transform);

        // Get position on hex grid
        let indices_opt = hex_grid.index(cursor_position.to_array());
        if let Some(indices) = indices_opt {
            let hex_item = hex_grid.get_mut(indices);

            // #[cfg(debug_assertions)]
            // println!(
            //     "indices: {:?}, selected_entity: {:?}, hex_item: {:?}",
            //     indices, selected_entity, hex_item
            // );

            match (selected_entity.0, hex_item) {
                // If we have an entity selected and we clicked an empty space.
                (Some(selected), Some(HexItem::Empty)) => {
                    // Attempt to move current entity to given position
                    // If position within movement

                    // #[cfg(debug_assertions)]
                    // println!(
                    //     "hex_grid[selected]({:?}): {:?}",
                    //     selected, hex_grid[selected]
                    // );

                    // Gets unit info and transform of this entity
                    let (_, unit, transform) = query.get_mut(hex_grid[selected].entity()).unwrap();
                    let unit = unit.into_inner();
                    // println!("unit: {:?}",unit);

                    // Gets distance to this new position (it will be `None` if the position is not reachable)
                    if let Some(&distance) = unit.movement_data.reachable_tiles.get(&indices) {
                        // println!("distance: {:?}->{:?}={}",selected,indices,distance);

                        // Gets logical pixel coordinates of new hex position
                        let [new_x, new_y] = hex_grid.logical_pixels(indices).unwrap();
                        // Moves entity to new hex
                        transform.into_inner().translation = Vec3::from((new_x, new_y, 1f32));
                        // Sets new hex entity ref
                        hex_grid[indices] = hex_grid[selected];
                        // Removes entity ref from old new
                        hex_grid[selected] = HexItem::Empty;
                        // Updates selected item
                        selected_entity.0 = Some(indices);
                        // Highlight new unit position
                        hex_grid
                            .highlight_cell(&mut commands, indices, UNIT_SELECTION_COLOUR)
                            .unwrap();

                        // Updates remaining movement
                        unit.remaining_time -= unit.movement_time * distance as f32;

                        // Movement range
                        clear_reachable(unit, &mut commands);
                        render_reachable(
                            unit,
                            &mut commands,
                            indices,
                            hex_grid,
                            &asset_server,
                            *camera_transform,
                        );
                        // Removes old firing lines

                        for line in firing_line.0.iter() {
                            commands.entity(*line).despawn();
                        }
                        firing_line.0 = Vec::new();

                        audio.set_volume(0.1);
                        audio.play(asset_server.load("PM_FN_Spawns_Portals_Teleports_5.mp3"));
                    } else {
                        println!("Cannot move: Outside unit's remaining movement points.");
                    }
                    // #[cfg(debug_assertions)]
                    // println!(
                    //     "hex_grid[indices]({:?}): {:?}, hex_grid[selected]({:?}): {:?}",
                    //     indices, hex_grid[indices], selected, hex_grid[selected]
                    // );
                }
                // If we clicked an entity.
                (_, Some(HexItem::Entity(_))) => {
                    // Highlight new unit position
                    hex_grid
                        .highlight_cell(&mut commands, indices, UNIT_SELECTION_COLOUR)
                        .unwrap();
                    // If we had previous unit selected clear the reachable areas on this unit
                    if let Some(old_index) = selected_entity.0 {
                        let (_, old_unit, _) = query.get_mut(hex_grid[old_index].entity()).unwrap();
                        clear_reachable(old_unit.into_inner(), &mut commands);
                    }
                    // Movement range
                    let (_, new_unit, _) = query.get_mut(hex_grid[indices].entity()).unwrap();

                    // println!("new_unit: {:?}",new_unit);
                    render_reachable(
                        new_unit.into_inner(),
                        &mut commands,
                        indices,
                        hex_grid,
                        &asset_server,
                        *camera_transform,
                    );
                    // Set new selected entity.
                    selected_entity.0 = Some(indices);
                    // Removes old firing lines
                    for line in firing_line.0.iter() {
                        commands.entity(*line).despawn();
                    }
                    firing_line.0 = Vec::new();
                }
                // If we don't have an entity and we didn't click and entity, do nothing
                _ => {}
            }
        }
    }
}

/// Clears all reachable tiles for a given [`Unit`].
fn clear_reachable(unit: &mut Unit, commands: &mut Commands) {
    unit.movement_data.clear(commands);
}

/// Spawns hex returning the entity.
fn spawn_hex(
    [x, y]: [f32; 2],
    fill_colour: Color,
    outline_colour: Color,
    commands: &mut Commands,
) -> Entity {
    commands
        .spawn_bundle(GeometryBuilder::build_as(
            &bevy_prototype_lyon::shapes::RegularPolygon {
                sides: 6,
                center: Vec2::new(x, y),
                feature: RegularPolygonFeature::SideLength(HEX_SIDE_LENGTH),
            },
            DrawMode::Outlined {
                fill_mode: FillMode::color(fill_colour),
                outline_mode: StrokeMode::new(outline_colour, HEX_OUTLINE_WIDTH),
            },
            Transform::default(),
        ))
        .id()
}

/// Spawns text returning the entity.
fn spawn_text(
    text: String,
    [x, y]: [f32; 2],
    font_size: f32,
    commands: &mut Commands,
    asset_server: &AssetServer,
) -> Entity {
    commands
        .spawn_bundle(Text2dBundle {
            text: Text::with_section(
                text,
                TextStyle {
                    font: asset_server.load("SmoochSans-Light.ttf"),
                    font_size,
                    color: Color::WHITE,
                },
                TextAlignment {
                    vertical: VerticalAlign::Center,
                    horizontal: HorizontalAlign::Center,
                },
            ),
            transform: Transform {
                translation: Vec3::new(x, y, 0f32),
                ..Default::default()
            },
            ..Default::default()
        })
        .id()
}

/// The colour used to highlight reachable tiles.
const REACHABLE_COLOR: Color = Color::rgba(0., 1., 0., 0.1);
/// The font size of unit remaining time displayed on reachable tiles.
const MOVEMENT_TIME_REMAINING_FONT_SIZE: f32 = 25f32;
/// The font size of possible remaining shots displayed on reachable tiles.
const MOVEMENT_SHOTS_REMAINING_FONT_SIZE: f32 = 40f32;
/// Spacing between possible shots and remaining unit time displayed on reachable tiles.
const REACHABLE_TEXT_SPACING: f32 = 2f32;
/// The zoom range within which to display remaining time units on a units reachable tiles.
const ZOOM_DISPLAY_TIME_RANGE: std::ops::RangeTo<f32> = ..1.5f32;
fn render_reachable(
    unit: &mut Unit,
    commands: &mut Commands,
    hex: [usize; 2],
    hex_grid: &HexGrid<HexItem>,
    asset_server: &AssetServer,
    camera_transform: Transform,
) {
    let max_movement = (unit.remaining_time / unit.movement_time) as usize;
    // Gets reachable positions
    let fringes = hex_grid.reachable(hex, max_movement);
    // Updates distances to all reachable positions
    let reachable_tiles = fringes
        .iter()
        .enumerate()
        .skip(1)
        .flat_map(|(distance, fringe)| {
            fringe
                .iter()
                .map(|hex| (*hex, distance))
                .collect::<Vec<_>>()
        })
        .collect::<HashMap<_, _>>();
    // Sets new movement range components
    let mut time_remaining_after_move = unit.remaining_time;

    let mut tile_highlights = Vec::new();
    let mut remaining_time = Vec::new();
    let mut possible_shots = Vec::new();

    for tiles in fringes.into_iter().skip(1) {
        time_remaining_after_move -= unit.movement_time;
        let possible_shots_after_move = time_remaining_after_move / unit.firing_time;
        for tile in tiles.into_iter() {
            let [x, y] = hex_grid.logical_pixels(tile).unwrap();
            tile_highlights.push(spawn_hex(
                [x, y],
                REACHABLE_COLOR,
                REACHABLE_COLOR,
                commands,
            ));
            possible_shots.push(spawn_text(
                (possible_shots_after_move as u32).to_string(),
                [x, y + MOVEMENT_TIME_REMAINING_FONT_SIZE / 2f32],
                MOVEMENT_SHOTS_REMAINING_FONT_SIZE,
                commands,
                asset_server,
            ));
            // If within zoom display range, display remaining unit time.
            if ZOOM_DISPLAY_TIME_RANGE.contains(&camera_transform.scale.to_array()[0]) {
                remaining_time.push(spawn_text(
                    (time_remaining_after_move as u32).to_string(),
                    [
                        x,
                        y - MOVEMENT_TIME_REMAINING_FONT_SIZE / 2f32 - REACHABLE_TEXT_SPACING,
                    ],
                    MOVEMENT_TIME_REMAINING_FONT_SIZE,
                    commands,
                    asset_server,
                ));
            }
        }
    }
    unit.movement_data = MovementData {
        reachable_tiles,
        tile_highlights,
        remaining_time,
        possible_shots,
    };
}

/// System handling camera movement
fn camera_movement_system(
    keys: Res<Input<KeyCode>>,
    mut scroll_events: EventReader<MouseWheel>,
    mut camera_query: Query<(&mut Transform, With<bevy::prelude::Camera>)>,
    mut unit_query: Query<&mut Unit>,
    mut commands: Commands,
    mut cursor_events: EventWriter<CursorMoved>,
    windows: Res<Windows>,
    asset_server: Res<AssetServer>,
    selected_entity: ResMut<SelectedUnitOption>,
    hex_grid: Res<HexGrid<HexItem>>,
) {
    // println!("\n");
    // TODO: We currently use `skip(1)` to skip the ui camera, is there not a better way to do this?
    let (camera_transform, _) = camera_query.iter_mut().nth(1).unwrap();
    let camera_transform = camera_transform.into_inner();
    if keys.pressed(KeyCode::W) || keys.pressed(KeyCode::Up) {
        // println!("up");
        // You cannot move the camera such that the center is off the hex grid.
        let temp = camera_transform.translation[1] + 10f32;
        if temp < hex_grid.logical_pixel_bounds[1].end {
            camera_transform.translation[1] = temp;
            // When moving the camera also update the highlighted text based on the new cursor
            //  position.
            trigger_cursor_event(&mut cursor_events, &windows);
        }
    }
    if keys.pressed(KeyCode::S) || keys.pressed(KeyCode::Down) {
        // println!("down");
        // You cannot move the camera such that the center is off the hex grid.
        let temp = camera_transform.translation[1] - 10f32;
        if temp > hex_grid.logical_pixel_bounds[1].start {
            camera_transform.translation[1] = temp;
            // When moving the camera also update the highlighted text based on the new cursor
            //  position.
            trigger_cursor_event(&mut cursor_events, &windows);
        }
    }
    if keys.pressed(KeyCode::A) || keys.pressed(KeyCode::Left) {
        // println!("left");
        // You cannot move the camera such that the center is off the hex grid.
        let temp = camera_transform.translation[0] - 10f32;
        if temp > hex_grid.logical_pixel_bounds[0].start {
            camera_transform.translation[0] = temp;
            // When moving the camera also update the highlighted text based on the new cursor
            //  position.
            trigger_cursor_event(&mut cursor_events, &windows);
        }
    }
    if keys.pressed(KeyCode::D) || keys.pressed(KeyCode::Right) {
        // println!("right");
        // You cannot move the camera such that the center is off the hex grid.
        let temp = camera_transform.translation[0] + 10f32;
        if temp < hex_grid.logical_pixel_bounds[0].end {
            camera_transform.translation[0] = temp;
            // When moving the camera also update the highlighted text based on the new cursor
            //  position.
            trigger_cursor_event(&mut cursor_events, &windows);
        }
    }
    for scroll_event in scroll_events.iter() {
        let temp = camera_transform.scale - (scroll_event.y / 10f32);
        // If new zoom is within limited zoom range, update camera zoom.
        if (MIN_ZOOM..MAX_ZOOM).contains(&temp[0]) {
            camera_transform.scale = temp;
            if let Some(selected_entity) = selected_entity.0 {
                let unit = unit_query
                    .get_mut(hex_grid[selected_entity].entity())
                    .unwrap()
                    .into_inner();
                // Within zoom display range for remaining time on tiles show this
                if ZOOM_DISPLAY_TIME_RANGE.contains(&temp[0]) {
                    unit.show_time(&hex_grid, &mut commands, &asset_server);
                } else {
                    unit.movement_data.clear_time(&mut commands);
                }
            }
        }
    }
    fn trigger_cursor_event(cursor_events: &mut EventWriter<CursorMoved>, windows: &Windows) {
        let window = windows.get_primary().expect("no primary window");
        if let Some(cursor_position) = window.cursor_position() {
            // #[cfg(debug_assertions)]
            // println!("cursor event sent");

            cursor_events.send(CursorMoved {
                id: window.id(),
                position: cursor_position,
            });
        }
    }
}

fn animation_system(
    time: Res<Time>,
    mut commands: Commands,
    mut unit_query: Query<(Entity, &mut Animation, &Transform)>,
) {
    unit_query.for_each_mut(|(entity, animation, transform)| {
        let animation = animation.into_inner();
        let delta = time.delta();
        if let Some(handle) = animation.tick(delta) {
            // Removes old sprite
            commands.entity(entity).remove_bundle::<SpriteBundle>();
            // println!("unit hit {:?}",delta);
            // Adds new sprite
            commands.entity(entity).insert_bundle(SpriteBundle {
                transform: *transform,
                texture: handle,
                visibility: bevy::render::view::Visibility { is_visible: true },
                ..Default::default()
            });
            // sprite_bundle.texture
        }
    });
}
/// Handles units firing weapons.
fn firing_system<const SHOT_SECONDS: f32, const SHOT_DECAY: f32>(
    time: Res<Time>,
    selected_entity: ResMut<SelectedUnitOption>,
    camera_query: Query<(&Transform, With<bevy::prelude::Camera>, Without<Unit>)>,
    mut unit_query: Query<(&mut Unit, &mut Transform)>,
    windows: Res<Windows>,
    hex_grid: ResMut<HexGrid<HexItem>>,
    mut commands: Commands,
    firing_line: ResMut<FiringLines>,
    #[cfg(debug_assertions)] firing_path: ResMut<FiringPath>,
    mut cursor_events: EventReader<CursorMoved>,
    asset_server: Res<AssetServer>,
    keys: Res<Input<KeyCode>>,
    lasers: ResMut<Lasers>,
    audio: Res<Audio>,
) {
    let firing_line = firing_line.into_inner();
    let hex_grid = hex_grid.into_inner();
    // Decays all rendered lasers and updates timers.
    let lasers = lasers.into_inner();
    lasers.0 = lasers
        .iter()
        .cloned()
        .filter_map(|mut laser| {
            // Ticks timer and checks if finished
            // println!("laser.timer.elapsed(): {:?}, {}",laser.timer.elapsed(),laser.timer.finished());
            if laser.timer.tick(time.delta()).just_finished() {
                // Removes old entity
                commands.entity(laser.entity).despawn();
                // println!("{} > {}",laser.decay,laser.opacity);
                if laser.decay > laser.opacity {
                    None
                } else {
                    laser.opacity -= laser.decay;
                    // Renders new entity
                    let mut shot_colour = LASER_COLOUR;
                    shot_colour.set_a(laser.opacity);

                    laser.entity = commands
                        .spawn_bundle(GeometryBuilder::build_as(
                            &bevy_prototype_lyon::shapes::Line(laser.from, laser.to),
                            DrawMode::Outlined {
                                fill_mode: FillMode::color(shot_colour),
                                outline_mode: StrokeMode::new(shot_colour, LASER_WIDTH),
                            },
                            Transform {
                                translation: Vec3::new(0f32, 0f32, 3f32),
                                ..Default::default()
                            },
                        ))
                        .id();
                    Some(laser)
                }
            } else {
                Some(laser)
            }
        })
        .collect::<Vec<_>>();

    // When the user presses the `f` key on their keyboard.
    if keys.just_pressed(KeyCode::F) {
        // We get the position of the cursor in the primary window.
        let window = windows.get_primary().expect("no primary window");
        if let Some(cursor_position) = window.cursor_position() {
            // If the user has some selected entity
            if let Some(selected) = selected_entity.0 {
                // Gets unit entity refers to.
                let unit = unit_query
                    .get_mut(hex_grid[selected].entity())
                    .unwrap()
                    .0
                    .into_inner();
                // If unit has enough time units left to fire.
                if unit.remaining_time >= unit.firing_time {
                    // Corrects cursor position
                    let cursor_position =
                        cursor_position - Vec2::new(window.width() / 2., window.height() / 2.);
                    let (camera_transform, _, _) = camera_query.iter().nth(1).unwrap();
                    let cursor_position =
                        normalize_cursor_position(cursor_position, camera_transform);
                    // Samples unit firing distribution getting the shot angle offset.
                    let theta = unit.firing_distribution.sample(&mut rand::thread_rng());
                    // Gets shot related data.
                    let [ax, ay] = hex_grid.logical_pixels(selected).unwrap();
                    let [bx, by] = cursor_position.to_array();
                    let [cx, cy] = rotate_point_around_point([ax, ay], [bx, by], theta);
                    let steps = 100;
                    let vector = Vec2::from([cx - ax, cy - ay]) / steps as f32;

                    // Calculates new firing path
                    let mut current = Vec2::from([ax, ay]);
                    // TODO We really only need to calculate distance(a,c) points. Do this instead (like https://www.redblobgames.com/grids/hexagons/#line-drawing).
                    let mut hex_path = HashSet::new();
                    loop {
                        current += vector;
                        // If indexes within grid can be given
                        if let Some(hex) = hex_grid.index(current.to_array()) {
                            // TODO Do this skip better.
                            // Skip hex if its the origin.
                            if hex == selected {
                                continue;
                            }
                            // If hex outside grid
                            if !hex_grid.contains_index(hex) {
                                break;
                            }
                            hex_path.insert(hex);

                            match hex_grid[hex] {
                                HexItem::Empty => {
                                    continue;
                                }
                                HexItem::Entity(entity) => {
                                    audio.set_volume(0.2);
                                    audio.play(asset_server.load("zapsplat_multimedia_game_sound_monster_hit_impact_kill_warp_weird_78163.mp3"));
                                    // TODO: Do better death animation
                                    // Removes hit entity
                                    hex_grid[hex] = HexItem::Empty;
                                    commands.entity(entity).despawn();

                                    break;
                                }
                                HexItem::Obstruction => {
                                    break;
                                }
                                HexItem::Enemy(_) => {
                                    break;
                                }
                            }
                        } else {
                            break;
                        }
                    }

                    // println!("collision: {:.2?}", collision);
                    // Only draw firing path hexes in debug
                    #[cfg(debug_assertions)]
                    {
                        let firing_path = firing_path.into_inner();
                        // Removes old firing path
                        for path in firing_path.0.iter() {
                            commands.entity(*path).despawn();
                        }
                        firing_path.0 = Vec::new();
                        // Adds firing path sprites
                        firing_path.0 = hex_path
                            .into_iter()
                            .map(|index| {
                                let hex = hex_grid.logical_pixels(index).unwrap();
                                spawn_hex(
                                    hex,
                                    FIRING_PATH_COLOUR,
                                    FIRING_PATH_COLOUR,
                                    &mut commands,
                                )
                            })
                            .collect::<Vec<_>>();
                    }

                    // TODO Fix so shot starts from edge of hex not center.
                    // Draws shot (since I don't want to handle animation a bullet, its a laser which appears for a couple frames)
                    let from = Vec2::from([ax, ay]);
                    // TODO Fix to so it doesn't end before hitting the grid edge or obstacle.
                    let to = current;
                    lasers.push(Laser {
                        entity: commands
                            .spawn_bundle(GeometryBuilder::build_as(
                                &bevy_prototype_lyon::shapes::Line(from, to),
                                DrawMode::Outlined {
                                    fill_mode: FillMode::color(LASER_COLOUR),
                                    outline_mode: StrokeMode::new(LASER_COLOUR, LASER_WIDTH),
                                },
                                Transform {
                                    translation: Vec3::new(0f32, 0f32, 3f32),
                                    ..Default::default()
                                },
                            ))
                            .id(),
                        decay: SHOT_DECAY,
                        opacity: LASER_COLOUR.a(),
                        timer: Timer::from_seconds(SHOT_SECONDS, true),
                        from,
                        to,
                    });
                    audio.set_volume(0.2);
                    audio.play(
                        asset_server
                            .load("cartoon_anime_burst_laser_beam_energy_fast_hard_71598.mp3"),
                    );
                    unit.remaining_time -= unit.firing_time;

                    // Movement range
                    clear_reachable(unit, &mut commands);
                    render_reachable(
                        unit,
                        &mut commands,
                        selected,
                        hex_grid,
                        &asset_server,
                        *camera_transform,
                    );
                }
            }
        }
    }
    // When the user moves their cursor we update the firing lines
    for cursor_event in cursor_events.iter() {
        // println!(
        //     "New cursor position: X: {}, Y: {}, in Window ID: {:?}",
        //     ev.position.x, ev.position.y, ev.id
        // );

        // Removes old firing lines
        for line in firing_line.0.iter() {
            commands.entity(*line).despawn();
        }
        firing_line.0 = Vec::new();

        if let Some(selected) = selected_entity.0 {
            let window = windows.get(cursor_event.id).unwrap();
            let cursor_position =
                cursor_event.position - Vec2::new(window.width() / 2., window.height() / 2.);
            let (camera_transform, _, _) = camera_query.iter().nth(1).unwrap();
            // Get cursor right position relative to camera
            let cursor_position = normalize_cursor_position(cursor_position, camera_transform);

            // Get the pixels corresponding to the center of the hex `selected` on the grid.
            let hex = hex_grid.logical_pixels(selected).unwrap();
            let extended_bounds = {
                let [x, y] = hex_grid.logical_pixel_bounds.clone();
                // We expand bounds so line is drawn to edge and doesn't end at edge hex
                let [w, h] = [HEX_WIDTH, HEX_HEIGHT];
                [x.start - w..x.end + w, y.start - h..y.end + h]
            };

            // Center firing line
            let center = end_point(
                hex,
                cursor_position.to_array(),
                extended_bounds.clone(),
                0f32,
            );

            let (unit, unit_transform) = unit_query.get_mut(hex_grid[selected].entity()).unwrap();
            let unit = unit.into_inner();

            // Calculates offset firing lines
            let ends_points  = unit.firing_buckets
                .iter()
                .flat_map(|angle|[
                    end_point(hex,cursor_position.to_array(),extended_bounds.clone(),-angle),
                    end_point(hex,cursor_position.to_array(),extended_bounds.clone(),*angle)
                ])
                .collect::<Vec<_>>();

            // #[cfg(debug_assertions)]
            // println!("updating firing lines");

            let draw = DrawMode::Outlined {
                fill_mode: FillMode::color(Color::rgba(0., 0., 0., 0.3)),
                outline_mode: StrokeMode::new(Color::rgba(0., 0., 0., 0.3), HEX_OUTLINE_WIDTH),
            };
            let transform = Transform {
                translation: Vec3::new(0f32, 0f32, FIRING_LINE_Z),
                ..Default::default()
            };
            assert!(transform.translation.is_finite(), "Firing line error");
            let mut lines = ends_points
                .into_iter()
                .map(|p| {
                    commands
                        .spawn_bundle(GeometryBuilder::build_as(
                            &bevy_prototype_lyon::shapes::Line(Vec2::from(hex), Vec2::from(p)),
                            draw.clone(),
                            transform.clone(),
                        ))
                        .id()
                })
                .collect::<Vec<_>>();

            // #[cfg(debug_assertions)]
            // println!("updated firing lines");

            // #[cfg(debug_assertions)]
            // println!("updating center firing line");

            let (from, to) = (Vec2::from(hex), Vec2::from(center));
            assert!(from.is_finite());
            assert!(to.is_finite());
            lines.push(
                commands
                    .spawn_bundle(GeometryBuilder::build_as(
                        &bevy_prototype_lyon::shapes::Line(from, to),
                        DrawMode::Outlined {
                            fill_mode: FillMode::color(Color::rgba(0., 0., 0., 0.1)),
                            outline_mode: StrokeMode::new(
                                Color::rgba(0., 0., 0., 0.1),
                                HEX_OUTLINE_WIDTH,
                            ),
                        },
                        transform.clone(),
                    ))
                    .id(),
            );
            // #[cfg(debug_assertions)]
            // println!("updated center firing lines");
           
            // Adds firing accuracies
            let firing_accuracy_text = commands
                .spawn_bundle(Text2dBundle {
                    text: Text::with_section(
                        format!(
                            "{}",
                            unit.firing_buckets.iter().map(|x|format!("{:.0}% ",x * 100f32)).collect::<String>()
                        ),
                        TextStyle {
                            font: asset_server.load("SmoochSans-Bold.ttf"),
                            font_size: 40.0,
                            color: Color::RED,
                        },
                        TextAlignment {
                            vertical: VerticalAlign::Center,
                            horizontal: HorizontalAlign::Center,
                        },
                    ),
                    transform: Transform {
                        translation: Vec3::new(hex[0], hex[1] + 50f32, 10f32),
                        ..Default::default()
                    },
                    ..Default::default()
                })
                .id();
            lines.push(firing_accuracy_text);
            firing_line.0 = lines;

            // Rotates sprite

            // #[cfg(debug_assertions)]
            // println!("updating unit angle");

            let [x, y] = cursor_position.to_array();
            let angle = (y - hex[1]).atan2(x - hex[0]);
            let offset_angle = angle + ANGLE_PINT_OFFSET;
            assert!(offset_angle.is_finite(), "Unit rotation error");
            unit_transform.into_inner().rotation = Quat::from_rotation_z(offset_angle);
            
            // #[cfg(debug_assertions)]
            // println!("updated unit firing angle");
        }
    }
}
// Offset add to selected unit sprite rotation relative to cursor position angle.
const ANGLE_PINT_OFFSET: f32 = std::f32::consts::PI / 2f32;

/// Returns a point on line from `a` through `c` (where `c` is `b` rotated about `a` by `theta` radians) that lies on the `bounds`.
///
/// <https://www.desmos.com/calculator/zdzfdwsjxa>
fn end_point(
    [ax, ay]: [f32; 2],
    [bx, by]: [f32; 2],
    [x, y]: [std::ops::Range<f32>; 2],
    theta: f32,
) -> [f32; 2] {
    let d = ((by - ay) / (bx - ax)).atan();
    let rad = (d + theta).tan();
    let fx = |x: f32| -> f32 { (rad * (x - ax)) + ay };
    let fy = |y: f32| -> f32 { ((y - ay) / rad) + ax };
    let [cx, cy] = rotate_point_around_point([ax, ay], [bx, by], theta);
    // println!(
    //     "{:?}->{:?} rotated by {:.3} gives {:?}",
    //     [ax, ay],[bx, by],theta,[cy,cx]
    // );
    match (cy > ay, cx > ax) {
        // Upper left quadrant
        (true, true) => [min(fy(y.end), x.end), min(y.end, fx(x.end))],
        // Lower left quadrant
        (false, true) => [min(fy(y.start), x.end), max(y.start, fx(x.end))],
        // Lower right quadrant
        (false, false) => [max(fy(y.start), x.start), max(y.start, fx(x.start))],
        // Upper right quadrant
        (true, false) => [max(fy(y.end), x.start), min(y.end, fx(x.start))],
    }
}

/// Rotates a given point `b` around a given point `a` by a some number of radians `theta`
fn rotate_point_around_point([ax, ay]: [f32; 2], [bx, by]: [f32; 2], theta: f32) -> [f32; 2] {
    let dx = bx - ax;
    let dy = by - ay;
    [
        dx * theta.cos() - dy * theta.sin() + ax,
        dx * theta.sin() + dy * theta.cos() + ay,
    ]
}

/// [`std::cmp::max`] supporting f32
fn max(a: f32, b: f32) -> f32 {
    if a > b {
        a
    } else {
        b
    }
}

/// [`std::cmp::min`] supporting f32
fn min(a: f32, b: f32) -> f32 {
    if a < b {
        a
    } else {
        b
    }
}

const DISTRIBUTION_BUCKETS: usize = 4;
const DISTRIBUTIONS_SAMPLES: usize = 100000;
const DISTRIBUTION_MAX_ANGLE: f32 = std::f32::consts::PI / 2f32;
const DISTRIBUTION_SAMPLES_STEP: f32 = DISTRIBUTION_MAX_ANGLE / DISTRIBUTION_BUCKETS as f32;

/// Returns the probability of sampling specific ranges of values from a given distribution.
fn buckets<D: rand_distr::Distribution<f32>>(dist: D) -> [f32; DISTRIBUTION_BUCKETS] {
    let mut buckets = [0; DISTRIBUTION_BUCKETS];
    // println!();
    for sample in dist.sample_iter(rand::thread_rng()).take(DISTRIBUTIONS_SAMPLES) {
        let abs_sample = sample.abs();
        // print!("{:.2}p,{} ",abs_sample / std::f32::consts::PI,(abs_sample / DISTRIBUTION_SAMPLES_STEP) as usize);
        let bucket = ((abs_sample / DISTRIBUTION_SAMPLES_STEP) as usize).clamp(0,DISTRIBUTION_BUCKETS);
        buckets[bucket] += 1;
    }
    println!("buckets buckets: {:?}",buckets);
    let mut percentage_buckets = [Default::default();DISTRIBUTION_BUCKETS];
    for i in 0..DISTRIBUTION_BUCKETS {
        percentage_buckets[i] = buckets[i] as f32 / DISTRIBUTIONS_SAMPLES as f32;
    }
    println!("buckets percentage_buckets: {:.3?}",percentage_buckets);
    percentage_buckets
}
