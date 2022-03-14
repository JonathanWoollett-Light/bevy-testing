#![allow(incomplete_features)]
#![feature(adt_const_params)]
use bevy::{input::mouse::MouseScrollUnit, prelude::*};
use bevy_prototype_lyon::prelude::*;
use std::collections::HashSet;
fn main() {
    App::new()
        .insert_resource(ClearColor(Color::rgb(0., 0., 0.)))
        .add_plugins(DefaultPlugins)
        .add_plugin(ShapePlugin)
        .add_startup_system(setup)
        .add_system(unit_movement_system::<{ [0, 0, 255, 50] }>)
        .add_system(camera_movement_system::<1f32, 5f32>)
        .add_system(hover_system::<{ [255, 255, 255, 127] }>)
        .add_system(turnover_system)
        .add_system(firing_system)
        .run();
}
use std::collections::HashMap;

#[derive(Debug, Clone)]
struct HexGrid<T: std::fmt::Debug> {
    data: Vec<T>,
    pub width: usize,
    pub height: usize,
    /// A mapping of RGBA 0..255 color highlights to their respective entity.
    highlights: HashMap<[u8; 4], Entity>,
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
        Self {
            data: vec![Default::default(); width * height],
            width,
            height,
            highlights: HashMap::new(),
        }
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

impl<T: std::fmt::Debug> HexGrid<T> {
    /// Returns if a given index is within bounds.
    fn contains_index(&self, [x, y]: [usize; 2]) -> bool {
        x < self.width && y < self.height
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
    ///  ___     ___
    /// ╱0,0╲___╱2,0╲___
    /// ╲___╱1,0╲___╱3,0╲
    /// ╱0,1╲___╱2,1╲___╱
    /// ╲___╱1,1╲___╱3,1╲
    ///     ╲___╱   ╲___╱
    ///
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
impl HexGrid<HexItem> {
    /// 3^0.5
    const SQRT_3: f32 = 1.7320508;
    /// sin(2*pi/3)
    const SIN_2_PI_BY_3: f32 = Self::SQRT_3 / 2f32;
    /// sin(pi/6)
    const SIN_PI_BY_6: f32 = 0.5f32;
    const LINE_WIDTH: f32 = 3f32;
    const SIDE_LENGTH: f32 = 30f32;
    const HEIGHT: f32 = Self::SIN_2_PI_BY_3 * (Self::SIDE_LENGTH / Self::SIN_PI_BY_6);
    const WIDTH: f32 = 3f32 * Self::SIDE_LENGTH / 2f32;
    /// Render hex grid on screen.
    fn render<const OPACITY: f32>(&self, commands: &mut Commands) {
        assert!((0f32..=1f32).contains(&OPACITY));
        for y in 0..self.height {
            for x in 0..self.width {
                let [center_x, center_y] = self.logical_pixel_coordinates([x, y]);
                // println!("[{},{}] -> [{:.1},{:.1}]",x,y,center_x,center_y);
                commands.spawn_bundle(GeometryBuilder::build_as(
                    &bevy_prototype_lyon::shapes::RegularPolygon {
                        sides: 6,
                        center: Vec2::new(center_x, center_y),
                        feature: RegularPolygonFeature::SideLength(Self::SIDE_LENGTH),
                    },
                    DrawMode::Outlined {
                        fill_mode: FillMode::color(Color::rgba(1., 1., 1., 0.3)),
                        outline_mode: StrokeMode::new(
                            Color::rgba(0., 0., 0., OPACITY),
                            Self::LINE_WIDTH,
                        ),
                    },
                    Transform::default(),
                ));
            }
        }
    }
    /// Gets logical pixel coordinates of center of hex of a given index.
    fn logical_pixel_coordinates(&self, [x, y]: [usize; 2]) -> [f32; 2] {
        [
            -(Self::WIDTH * self.width as f32 / 2f32) + Self::WIDTH * x as f32,
            -(Self::HEIGHT * self.height as f32 / 2f32)
                + (Self::HEIGHT * y as f32)
                + ((x % 2) as f32 * Self::HEIGHT / 2f32),
        ]
    }
    /// Gets logical pixel bounds of hex grid.
    fn logical_pixel_bounds(&self) -> [std::ops::Range<f32>; 2] {
        let [xs, ys] = self.logical_pixel_coordinates([0, 0]);
        let [xe, ye] = self.logical_pixel_coordinates([self.width - 1, self.height - 1]);
        [xs..xe, ys..ye]
    }
    const UNIT_SIZE: f32 = 20f32;
    fn add_component(
        &mut self,
        commands: &mut Commands,
        component: impl Component,
        index: [usize; 2],
    ) {
        let [x, y] = self.logical_pixel_coordinates(index);
        let mut new_entity_commands = commands.spawn_bundle(SpriteBundle {
            transform: Transform {
                translation: Vec3::from((x, y, 0f32)),
                scale: Vec3::new(Self::UNIT_SIZE, Self::UNIT_SIZE, 0.),
                ..Default::default()
            },
            sprite: Sprite {
                color: Color::rgb(1., 1., 1.),
                ..Default::default()
            },
            ..Default::default()
        });
        let new_entity = new_entity_commands.id();
        new_entity_commands.insert(component);
        match self.get_mut(index) {
            Some(x) => {
                *x = HexItem::Entity(new_entity);
            }
            None => unreachable!(),
        }
    }
    /// Gets indices from logical pixel coordinates.
    fn indices(&self, [x, y]: [f32; 2]) -> Option<[usize; 2]> {
        let i = ((x + Self::WIDTH / 2f32) + (Self::WIDTH * self.width as f32) / 2f32) / Self::WIDTH;
        let j = ((Self::HEIGHT * self.height as f32 / 2f32)
            - ((i as usize % 2) as f32 * Self::HEIGHT / 2f32)
            + (y + Self::HEIGHT / 2f32))
            / Self::HEIGHT;
        // println!("[{:.1},{:.1}]->[{:.1},{:.1}]", x, y, i, j);
        match i < 0f32 || j < 0f32 {
            true => None,
            false => Some([i as usize, j as usize]),
        }
    }
    /// Removes current highlighted cell.
    fn remove_highlight<const COLOR: [u8; 4]>(&mut self, commands: &mut Commands) {
        if let Some(existing_highlight) = self.highlights.remove(&COLOR) {
            commands.entity(existing_highlight).despawn();
        }
    }
    /// Highlights specific hex.
    fn highlight_cell<const COLOR: [u8; 4]>(&mut self, commands: &mut Commands, index: [usize; 2]) {
        self.remove_highlight::<COLOR>(commands);
        if self.contains_index(index) {
            let [x, y] = self.logical_pixel_coordinates(index);
            let color = Color::rgba(
                COLOR[0] as f32 / 255.,
                COLOR[1] as f32 / 255.,
                COLOR[2] as f32 / 255.,
                COLOR[3] as f32 / 255.,
            );
            let highlight = commands.spawn_bundle(GeometryBuilder::build_as(
                &bevy_prototype_lyon::shapes::RegularPolygon {
                    sides: 6,
                    center: Vec2::new(x, y),
                    feature: RegularPolygonFeature::SideLength(Self::SIDE_LENGTH),
                },
                DrawMode::Outlined {
                    fill_mode: FillMode::color(color),
                    outline_mode: StrokeMode::new(color, Self::LINE_WIDTH),
                },
                Transform::default(),
            ));
            self.highlights.insert(COLOR, highlight.id());
        }
    }
    /// Returns a 2d vec of reachable indices from the given index (where vec[x][2] denotes the an index reachable in `x` steps).
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
// impl From<&HexGrid<HexItem>> for HexGrid<bool> {
//     fn from(a: &HexGrid<HexItem>) -> Self {
//         HexGrid::new(a.width,a.height)
//     }
// }

#[derive(Debug, Copy, Clone)]
enum HexItem {
    Entity(Entity),
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
fn setup(mut commands: Commands) {
    // cameras
    commands.spawn_bundle(OrthographicCameraBundle::new_2d());
    commands.spawn_bundle(UiCameraBundle::default());

    // Background
    let grid_height = 20;
    let grid_width = 39;

    let mut hex_grid = HexGrid::new(grid_width, grid_height);
    hex_grid.render::<0.1f32>(&mut commands);

    let units = vec![
        (Unit::new(1), [1, 1]),
        (Unit::new(2), [2, 2]),
        (Unit::new(3), [3, 3]),
        (Unit::new(4), [4, 4]),
    ];

    commands.insert_resource(SelectedUnitOption::default());
    commands.insert_resource(FiringLines::default());

    // Add units
    // ----------------------------------------------------

    for (unit, index) in units.into_iter() {
        hex_grid.add_component(&mut commands, unit, index);
    }
    commands.insert_resource(hex_grid);
}
#[derive(Default, Debug)]
struct FiringLines(Vec<Entity>);
#[derive(Default, Debug)]
struct SelectedUnitOption(Option<[usize; 2]>);
#[derive(Component, Debug)]
struct Unit {
    firing_buckets: [f32; 3],
    firing_distribution: rand_distr::Normal<f32>,
    // Distances to all reachable points from current position.
    distances: HashMap<[usize; 2], usize>,
    movement_range: Vec<Entity>,
    movement: u8,
    maximum_movement: u8,
}
impl Unit {
    const SAMPLES: usize = 10000;
    fn new(
        maximum_movement: u8,
        // map:&HexGrid<HexItem>
    ) -> Self {
        let firing_distribution = rand_distr::Normal::new(0f32, 0.07f32).unwrap();
        Self {
            firing_buckets: buckets(firing_distribution, Self::SAMPLES),
            firing_distribution,
            distances: HashMap::new(),
            movement_range: Vec::new(),
            maximum_movement,
            movement: maximum_movement,
        }
    }
}

fn hover_system<const HIGHLIGHT_COLOR: [u8; 4]>(
    windows: Res<Windows>,
    mut commands: Commands,
    mut cursor_evr: EventReader<CursorMoved>,
    hex_grid: ResMut<HexGrid<HexItem>>,
    camera_query: Query<(&Transform, With<bevy::prelude::Camera>)>,
) {
    let hex_grid = hex_grid.into_inner();
    for _ in cursor_evr.iter() {
        let window = windows.get_primary().expect("no primary window");
        let cursor_position = window.cursor_position().expect("No cursor position")
            - Vec2::new(window.width() / 2., window.height() / 2.);
        // print!("{}",cursor_position);

        // TODO Here we skip our 1st camera (the ui camera) do this better
        let (camera_transform, _) = camera_query.iter().nth(1).unwrap();
        let cursor_position = normalize_cursor_position(cursor_position, camera_transform);
        // println!("cursor_position: {:?}", cursor_position);

        let indices = hex_grid.indices(cursor_position.to_array());
        // println!("hover: {:?}",indices);
        match indices {
            // If both logical pixel coordinates can be mapped to hexes within our hex grid
            Some([x, y]) => {
                hex_grid.highlight_cell::<HIGHLIGHT_COLOR>(&mut commands, [x, y]);
            }
            // If either logical pixel coordinates are outside our hex grid
            _ => {
                hex_grid.remove_highlight::<HIGHLIGHT_COLOR>(&mut commands);
            }
        }
    }
}

// Scales and translates cursor position relative to camera getting global cursor position.
fn normalize_cursor_position(pos: Vec2, camera_transform: &Transform) -> Vec2 {
    (pos * camera_transform.scale.truncate()) + camera_transform.translation.truncate()
}
fn turnover_system(
    keys: Res<Input<KeyCode>>,
    mut query: Query<(&mut Unit)>,
    selected_entity: ResMut<SelectedUnitOption>,
    hex_grid: ResMut<HexGrid<HexItem>>,
    mut commands: Commands,
    asset_server: Res<AssetServer>,
) {
    if keys.just_pressed(KeyCode::Space) {
        query.for_each_mut(|mut f| {
            f.movement = f.maximum_movement;
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
                asset_server,
            );
        }
    }
}

/// System handling unit movement
fn unit_movement_system<const HIGHLIGHT_COLOR: [u8; 4]>(
    selected_entity: ResMut<SelectedUnitOption>,
    mut commands: Commands,
    buttons: Res<Input<MouseButton>>,
    mut query: Query<(Entity, &mut Unit, &mut Transform)>,
    windows: Res<Windows>,
    hex_grid: ResMut<HexGrid<HexItem>>,
    camera_query: Query<(&Transform, With<bevy::prelude::Camera>, Without<Unit>)>,
    asset_server: Res<AssetServer>,
    firing_line: ResMut<FiringLines>,
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
            // De-spawns old movement range components
            for entity in unit.movement_range.iter() {
                commands.entity(*entity).despawn();
            }
            unit.movement_range = Vec::new();
        }
        // Remove selected unit
        selected_entity.0 = None;
        // Remove unit position highlight
        hex_grid.remove_highlight::<HIGHLIGHT_COLOR>(&mut commands);

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
        let indices_opt = hex_grid.indices(cursor_position.to_array());
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
                    if let Some(distance) = unit.distances.get(&indices) {
                        // println!("distance: {:?}->{:?}={}",selected,indices,distance);

                        // Gets logical pixel coordinates of new hex position
                        let [new_x, new_y] = hex_grid.logical_pixel_coordinates(indices);
                        // Moves entity to new hex
                        transform.into_inner().translation = Vec3::from((new_x, new_y, 0f32));
                        // Sets new hex entity ref
                        hex_grid[indices] = hex_grid[selected];
                        // Removes entity ref from old new
                        hex_grid[selected] = HexItem::Empty;
                        // Updates selected item
                        selected_entity.0 = Some(indices);
                        // Highlight new unit position
                        hex_grid.highlight_cell::<HIGHLIGHT_COLOR>(&mut commands, indices);

                        // Updates remaining movement
                        unit.movement = unit
                            .movement
                            .checked_sub(u8::try_from(*distance).unwrap())
                            .unwrap();

                        // Movement range
                        clear_reachable(unit, &mut commands);
                        render_reachable(unit, &mut commands, indices, hex_grid, asset_server);
                        // Removes old firing lines

                        for line in firing_line.0.iter() {
                            commands.entity(*line).despawn();
                        }
                        firing_line.0 = Vec::new();
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
                    hex_grid.highlight_cell::<HIGHLIGHT_COLOR>(&mut commands, indices);
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
                        asset_server,
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
fn clear_reachable(unit: &mut Unit, commands: &mut Commands) {
    for entity in unit.movement_range.iter() {
        commands.entity(*entity).despawn();
    }
    unit.movement_range = Vec::new();
}
fn render_reachable(
    new_unit: &mut Unit,
    commands: &mut Commands,
    hex: [usize; 2],
    hex_grid: &HexGrid<HexItem>,
    asset_server: Res<AssetServer>,
) {
    // Gets reachable positions
    let fringes = hex_grid.reachable(hex, new_unit.movement as usize);
    // Updates distances to all reachable positions
    let distances = fringes
        .iter()
        .enumerate()
        .flat_map(|(distance, fringe)| {
            fringe
                .iter()
                .map(|hex| (*hex, distance))
                .collect::<Vec<_>>()
        })
        .collect::<HashMap<_, _>>();
    new_unit.distances = distances;
    // Sets new movement range components
    let highlight_entities = fringes
        .into_iter()
        .enumerate()
        .skip(1)
        .flat_map(|(distance, hexes)| {
            hexes
                .into_iter()
                .flat_map(|reachable_hex| {
                    let [x, y] = hex_grid.logical_pixel_coordinates(reachable_hex);
                    // println!("pos: {:?}", [x, y]);
                    let highlight = commands
                        .spawn_bundle(GeometryBuilder::build_as(
                            &bevy_prototype_lyon::shapes::RegularPolygon {
                                sides: 6,
                                center: Vec2::new(x, y),
                                feature: RegularPolygonFeature::SideLength(
                                    HexGrid::<HexItem>::SIDE_LENGTH,
                                ),
                            },
                            DrawMode::Outlined {
                                fill_mode: FillMode::color(Color::rgba(0., 1., 0., 0.3)),
                                outline_mode: StrokeMode::new(
                                    Color::rgba(0., 1., 0., 0.3),
                                    HexGrid::<HexItem>::LINE_WIDTH,
                                ),
                            },
                            Transform::default(),
                        ))
                        .id();
                    let distance_text = commands
                        .spawn_bundle(Text2dBundle {
                            text: Text::with_section(
                                distance.to_string(),
                                TextStyle {
                                    font: asset_server.load("SmoochSans-Light.ttf"),
                                    font_size: 60.0,
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
                        .id();
                    [highlight, distance_text]
                })
                .collect::<Vec<_>>()
        })
        .collect::<Vec<_>>();
    new_unit.movement_range = highlight_entities;
}

use bevy::input::mouse::MouseWheel;
/// System handling camera movement
fn camera_movement_system<const MIN_ZOOM: f32, const MAX_ZOOM: f32>(
    keys: Res<Input<KeyCode>>,
    mut scroll_evr: EventReader<MouseWheel>,
    mut query: Query<(&mut Transform, With<bevy::prelude::Camera>)>,
    hex_grid: Res<HexGrid<HexItem>>,
) {
    // println!("\n");
    // TODO: We currently use `skip(1)` to skip the ui camera, is there not a better way to do this?
    for (transform, _) in query.iter_mut().skip(1) {
        let transform = transform.into_inner();
        if keys.pressed(KeyCode::W) || keys.pressed(KeyCode::Up) {
            // println!("up");
            // You cannot move the camera such that the center is off the hex grid.
            let temp = transform.translation[1] + 10f32;
            if temp < hex_grid.logical_pixel_bounds()[1].end {
                transform.translation[1] = temp;
            }
        }
        if keys.pressed(KeyCode::S) || keys.pressed(KeyCode::Down) {
            // println!("down");
            // You cannot move the camera such that the center is off the hex grid.
            let temp = transform.translation[1] - 10f32;
            if temp > hex_grid.logical_pixel_bounds()[1].start {
                transform.translation[1] = temp;
            }
        }
        if keys.pressed(KeyCode::A) || keys.pressed(KeyCode::Left) {
            // println!("left");
            // You cannot move the camera such that the center is off the hex grid.
            let temp = transform.translation[0] - 10f32;
            if temp > hex_grid.logical_pixel_bounds()[0].start {
                transform.translation[0] = temp;
            }
        }
        if keys.pressed(KeyCode::D) || keys.pressed(KeyCode::Right) {
            // println!("right");
            // You cannot move the camera such that the center is off the hex grid.
            let temp = transform.translation[0] + 10f32;
            if temp < hex_grid.logical_pixel_bounds()[0].end {
                transform.translation[0] = temp;
            }
        }
        for ev in scroll_evr.iter() {
            match ev.unit {
                MouseScrollUnit::Line => {
                    // println!("camera: {:?}",camera);
                    // println!("Scroll (line units): vertical: {}, horizontal: {}", ev.y, ev.x);
                    let temp = transform.scale - (ev.y / 10f32);
                    if temp[0] >= MIN_ZOOM && temp[0] <= MAX_ZOOM {
                        transform.scale = temp;
                    }
                }
                MouseScrollUnit::Pixel => {
                    // println!("Scroll (pixel units): vertical: {}, horizontal: {}", ev.y, ev.x);
                    let temp = transform.scale - (ev.y / 10f32);
                    if temp[0] >= MIN_ZOOM && temp[0] <= MAX_ZOOM {
                        transform.scale = temp;
                    }
                }
            }
        }
    }
}

fn firing_system(
    selected_entity: ResMut<SelectedUnitOption>,
    camera_query: Query<(&Transform, With<bevy::prelude::Camera>, Without<Unit>)>,
    unit_query: Query<(&Unit)>,
    windows: Res<Windows>,
    hex_grid: Res<HexGrid<HexItem>>,
    mut commands: Commands,
    firing_line: ResMut<FiringLines>,
    mut cursor_evr: EventReader<CursorMoved>,
    asset_server: Res<AssetServer>,
) {
    let firing_line = firing_line.into_inner();
    // TODO Use cursor position from `_ev` instead of `window`.
    for _ev in cursor_evr.iter() {
        // println!(
        //     "New cursor position: X: {}, Y: {}, in Window ID: {:?}",
        //     ev.position.x, ev.position.y, ev.id
        // );

        // Removes old firing lines
        for line in firing_line.0.iter() {
            commands.entity(*line).despawn();
        }
        firing_line.0 = Vec::new();

        let window = windows.get_primary().expect("no primary window");
        // If cursor has position in window
        if let Some(cursor_position) = window.cursor_position() {
            let cursor_position =
                cursor_position - Vec2::new(window.width() / 2., window.height() / 2.);
            let (camera_transform, _, _) = camera_query.iter().nth(1).unwrap();
            // Get cursor right position relative to camera
            let cursor_position = normalize_cursor_position(cursor_position, camera_transform);
            if let Some(selected) = selected_entity.0 {
                // Get the pixels corresponding to the center of the hex `selected` on the grid.
                let hex = hex_grid.logical_pixel_coordinates(selected);
                let bounds = hex_grid.logical_pixel_bounds();
                let extended_bounds = {
                    let [x, y] = bounds;
                    // We expand bounds so line is drawn to edge and doesn't end at edge hex
                    let [w, h] = [HexGrid::<HexItem>::WIDTH, HexGrid::<HexItem>::HEIGHT];
                    [x.start - w..x.end + w, y.start - h..y.end + h]
                };
                // Calculate end point

                let center = end_point(
                    hex,
                    cursor_position.to_array(),
                    extended_bounds.clone(),
                    0f32,
                );
                let points = vec![
                    end_point(
                        hex,
                        cursor_position.to_array(),
                        extended_bounds.clone(),
                        0.02f32,
                    ),
                    end_point(
                        hex,
                        cursor_position.to_array(),
                        extended_bounds.clone(),
                        -0.02f32,
                    ),
                    end_point(
                        hex,
                        cursor_position.to_array(),
                        extended_bounds.clone(),
                        0.1f32,
                    ),
                    end_point(
                        hex,
                        cursor_position.to_array(),
                        extended_bounds.clone(),
                        -0.1f32,
                    ),
                ];

                let draw = DrawMode::Outlined {
                    fill_mode: FillMode::color(Color::rgba(0., 0., 0., 0.3)),
                    outline_mode: StrokeMode::new(
                        Color::rgba(0., 0., 0., 0.3),
                        HexGrid::<HexItem>::LINE_WIDTH,
                    ),
                };
                let transform = Transform {
                    translation: Vec3::new(0f32, 0f32, 2f32),
                    ..Default::default()
                };
                let mut lines = points
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
                lines.push(
                    commands
                        .spawn_bundle(GeometryBuilder::build_as(
                            &bevy_prototype_lyon::shapes::Line(Vec2::from(hex), Vec2::from(center)),
                            DrawMode::Outlined {
                                fill_mode: FillMode::color(Color::rgba(0., 0., 0., 0.1)),
                                outline_mode: StrokeMode::new(
                                    Color::rgba(0., 0., 0., 0.1),
                                    HexGrid::<HexItem>::LINE_WIDTH,
                                ),
                            },
                            transform.clone(),
                        ))
                        .id(),
                );
                let unit = unit_query.get(hex_grid[selected].entity()).unwrap();
                let firing_accuracy_text = commands
                    .spawn_bundle(Text2dBundle {
                        text: Text::with_section(
                            format!(
                                "{:.0}% {:.0}% {:.0}%",
                                unit.firing_buckets[0] * 100f32,
                                unit.firing_buckets[1] * 100f32,
                                unit.firing_buckets[2] * 100f32
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
            }
        }
    }
}
fn angle_between_two_points([ax, ay]: [f32; 2], [bx, by]: [f32; 2]) -> f32 {
    let delta_x = ax - bx;
    let delta_y = ay - by;
    let radians = delta_y.atan2(delta_x);
    radians
}
fn cartesian_distance([ax, ay]: [f32; 2], [bx, by]: [f32; 2]) -> f32 {
    ((ax as f32 - bx as f32).powi(2) + (ay as f32 - by as f32).powi(2)).sqrt()
}
/// Returns a point on line from `a` through `c` (where `c` is `b` rotated about `a` by `theta` radians) that lies on the `bounds`.
///
/// https://www.desmos.com/calculator/zdzfdwsjxa
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
// std::cmp::max supporting f32
fn max(a: f32, b: f32) -> f32 {
    if a > b {
        a
    } else {
        b
    }
}
// std::cmp::min supporting f32
fn min(a: f32, b: f32) -> f32 {
    if a < b {
        a
    } else {
        b
    }
}
// Returns the probability of sampling ranges of values from a distribution
// The buckets being:
// - -0.02..0.02
// - -0.1..-0.02 & 0.02..0.1
// - ..-0.1 & 0.1..
fn buckets<D: rand_distr::Distribution<f32>>(dist: D, samples: usize) -> [f32; 3] {
    let mut buckets = [0; 3];
    for sample in dist.sample_iter(rand::thread_rng()).take(samples) {
        if sample < -0.1 || sample > 0.1 {
            buckets[2] += 1;
        } else if sample < -0.02 || sample > 0.02 {
            buckets[1] += 1;
        } else {
            buckets[0] += 1;
        }
    }
    [
        buckets[0] as f32 / samples as f32,
        buckets[1] as f32 / samples as f32,
        buckets[2] as f32 / samples as f32,
    ]
}
