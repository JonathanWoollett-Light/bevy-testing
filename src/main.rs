#![allow(incomplete_features)]
#![feature(adt_const_params)]
#![feature(const_fn_floating_point_arithmetic)]
use bevy::{input::mouse::MouseWheel, prelude::*};
use bevy_kira_audio::{Audio, AudioPlugin};
use bevy_prototype_lyon::prelude::*;
use rand_distr::Distribution;
use std::collections::{HashMap, HashSet};

const FIRING_LINE_ALPHA: f32 = 0.8;
const GRADIENT_DETAIL: usize = 1000;
/// Alpha of center aiming line
const AIMING_LINE_ALPHA: f32 = 0.;
const DISTRIBUTION_BUCKETS: usize = 500;
const DISTRIBUTIONS_SAMPLES: usize = 100000;
const DISTRIBUTION_SAMPLES_STEP: f32 = 0.002f32;
const MAX_FIRE_ANGLE: f32 = (DISTRIBUTION_BUCKETS - 1) as f32 * DISTRIBUTION_SAMPLES_STEP;

mod models;
use models::*;

// TODO Make this const
lazy_static::lazy_static! {
    /// This is an example for using doc comment attributes
    static ref GRADIENT: [[f32;4];GRADIENT_DETAIL] = gradient::<GRADIENT_DETAIL>([0.2,0.2,0.2,0.],[0.8666,0.0941,0.0941,FIRING_LINE_ALPHA]);
}

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
/// Colour of outline of hexagons used to form the map.
const HEX_OUTLINE_COLOR: Color = Color::rgba(0.2f32, 0.2f32, 0.2f32, 1f32);
/// Samples to take from firing distributions to approximate firing accuracies.
const SAMPLES: usize = 10;

/// Firing lines colour.
const FIRING_PATH_COLOUR: Color = Color::rgba(1., 1., 1., 0.2);
/// Z position of firing lines.
const FIRING_LINE_Z: f32 = 5f32;
/// Width of laser rectangles.
const LASER_WIDTH: f32 = 3f32;

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

    // Add units
    // ----------------------------------------------------

    #[cfg(debug_assertions)]
    println!("started spawning units");

    for index in map.player_spawns.iter() {
        hex_grid.add_unit(&mut commands, *index, asset_server);
    }
    #[cfg(debug_assertions)]
    println!("spawned units");

    // #[cfg(debug_assertions)]
    // println!("started spawning enemies");
    // for (enemy, index) in enemy_units.into_iter() {
    //     hex_grid.add_enemy(&mut commands, enemy, index, asset_server);
    // }
    // #[cfg(debug_assertions)]
    // println!("spawned enemies");

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
    mut firing_spread_query: Query<(Entity, &mut FiringSpread, &mut Transform, Without<Unit>)>,
    windows: Res<Windows>,
    hex_grid: ResMut<HexGrid<HexItem>>,
    camera_query: Query<(
        &Transform,
        With<bevy::prelude::Camera>,
        Without<FiringSpread>,
        Without<Unit>,
    )>,
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

            // Hide accuracy field
            let (_, _, firing_transform, _) =
                firing_spread_query.get_mut(unit.accuracy_field).unwrap();
            let firing_transform = firing_transform.into_inner();
            firing_transform.translation = Vec3::new(
                firing_transform.translation[0],
                firing_transform.translation[1],
                -1f32,
            );

            // De-spawns old movement components
            clear_reachable(unit, &mut commands);
        }
        // Remove selected unit
        selected_entity.0 = None;
        // Remove unit position highlight
        hex_grid.remove_highlight(&mut commands, UNIT_SELECTION_COLOUR);
    }
    if buttons.just_pressed(MouseButton::Left) {
        // #[cfg(debug_assertions)]
        // println!("left click");

        if let Some(mut cursor_position) = window.cursor_position() {
            cursor_position -= Vec2::new(window.width() / 2., window.height() / 2.);
            // TODO Here we effectively skip ui camera, do this better
            let (camera_transform, _, _, _) = camera_query.iter().nth(1).unwrap();
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
                        let (_, unit, transform) =
                            query.get_mut(hex_grid[selected].entity()).unwrap();
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
                            // Play movement sound
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
                            let (_, old_unit, _) =
                                query.get_mut(hex_grid[old_index].entity()).unwrap();
                            let old_unit = old_unit.into_inner();
                            clear_reachable(old_unit, &mut commands);

                            // Hides old accuracy field
                            let (_, _, firing_transform, _) = firing_spread_query
                                .get_mut(old_unit.accuracy_field)
                                .unwrap();
                            let firing_transform = firing_transform.into_inner();
                            firing_transform.translation = Vec3::new(
                                firing_transform.translation[0],
                                firing_transform.translation[1],
                                -1f32,
                            );
                        }
                        // Movement range
                        let (_, new_unit, _) = query.get_mut(hex_grid[indices].entity()).unwrap();
                        let new_unit = new_unit.into_inner();

                        // Show accuracy field
                        let (_, _, firing_transform, _) = firing_spread_query
                            .get_mut(new_unit.accuracy_field)
                            .unwrap();
                        let firing_transform = firing_transform.into_inner();
                        firing_transform.translation = Vec3::new(
                            firing_transform.translation[0],
                            firing_transform.translation[1],
                            1f32,
                        );

                        // println!("new_unit: {:?}",new_unit);
                        render_reachable(
                            new_unit,
                            &mut commands,
                            indices,
                            hex_grid,
                            &asset_server,
                            *camera_transform,
                        );
                        // Set new selected entity.
                        selected_entity.0 = Some(indices);
                    }
                    // If we don't have an entity and we didn't click and entity, do nothing
                    _ => {}
                }
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
    camera_query: Query<(
        &Transform,
        With<bevy::prelude::Camera>,
        Without<Unit>,
        Without<FiringSpread>,
    )>,
    mut unit_query: Query<(&mut Unit, &mut Transform, Without<FiringSpread>)>,
    mut firing_spread_query: Query<(&mut FiringSpread, &mut Transform, Without<Unit>)>,
    windows: Res<Windows>,
    hex_grid: ResMut<HexGrid<HexItem>>,
    mut commands: Commands,
    #[cfg(debug_assertions)] firing_path: ResMut<FiringPath>,
    mut cursor_events: EventReader<CursorMoved>,
    asset_server: Res<AssetServer>,
    keys: Res<Input<KeyCode>>,
    lasers: ResMut<Lasers>,
    audio: Res<Audio>,
) {
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
                    let (camera_transform, _, _, _) = camera_query.iter().nth(1).unwrap();
                    let cursor_position =
                        normalize_cursor_position(cursor_position, camera_transform);
                    // Samples unit firing distribution getting the shot angle offset.
                    let theta = unit
                        .firing_distribution
                        .sample(&mut rand::thread_rng())
                        .clamp(-MAX_FIRE_ANGLE, MAX_FIRE_ANGLE);
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

        if let Some(selected) = selected_entity.0 {
            let window = windows.get(cursor_event.id).unwrap();
            let cursor_position =
                cursor_event.position - Vec2::new(window.width() / 2., window.height() / 2.);
            let (camera_transform, _, _, _) = camera_query.iter().nth(1).unwrap();
            // Get cursor right position relative to camera
            let cursor_position = normalize_cursor_position(cursor_position, camera_transform);

            // Get the pixels corresponding to the center of the hex `selected` on the grid.
            let hex = hex_grid.logical_pixels(selected).unwrap();

            let (unit, unit_transform, _) =
                unit_query.get_mut(hex_grid[selected].entity()).unwrap();
            let (_, firing_spread_transform, _) =
                firing_spread_query.get_mut(unit.accuracy_field).unwrap();

            // Rotates units
            let [x, y] = cursor_position.to_array();
            let angle = (y - hex[1]).atan2(x - hex[0]);
            let offset_angle = angle + ANGLE_PINT_OFFSET;
            assert!(offset_angle.is_finite(), "Unit rotation error");
            let unit_transform = unit_transform.into_inner();
            unit_transform.rotation = Quat::from_rotation_z(offset_angle);

            let firing_spread_transform = firing_spread_transform.into_inner();
            firing_spread_transform.rotation = Quat::from_rotation_z(angle);

            let target_pos = Vec2::new(x, y);

            let sprite_pos = unit_transform.translation.truncate()
                + (Vec2::normalize(target_pos - unit_transform.translation.truncate())
                    * (FIRING_SPREAD_WIDTH as f32 / 2f32));

            firing_spread_transform.translation = Vec3::from((sprite_pos, 10f32));

            println!("firing_spread_transform.translation: {:?}\n firing_spread_transform.rotation: {:?}",firing_spread_transform.translation, firing_spread_transform.rotation);
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
fn end_point_unbound([ax, ay]: [f32; 2], [bx, by]: [f32; 2], theta: f32, length: f32) -> [f32; 2] {
    let d = ((by - ay) / (bx - ax)).atan();
    let rad = (d + theta).tan();
    let fx = |x: f32| -> f32 { (rad * (x - ax)) + ay };
    // let fy = |y: f32| -> f32 { ((y - ay) / rad) + ax };
    let [cx, cy] = rotate_point_around_point([ax, ay], [bx, by], theta);
    let dx = ax + length;
    [dx, fx(dx)]
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

/// Returns the probability of sampling specific values in given ranges (where ranges are presumed to be non-overlapping).
fn buckets<D: rand_distr::Distribution<f32>>(dist: D) -> [f32; DISTRIBUTION_BUCKETS] {
    let buckets = {
        let mut buckets = [0u32; DISTRIBUTION_BUCKETS];
        for sample in dist
            .sample_iter(rand::thread_rng())
            .take(DISTRIBUTIONS_SAMPLES)
        {
            let abs_sample = sample.abs().clamp(0f32, MAX_FIRE_ANGLE);
            // print!("{:.2}p,{} ",abs_sample / std::f32::consts::PI,(abs_sample / DISTRIBUTION_SAMPLES_STEP) as usize);
            let bucket = (abs_sample / DISTRIBUTION_SAMPLES_STEP) as usize;
            assert!(bucket < DISTRIBUTION_BUCKETS);
            buckets[bucket] += 1;
        }
        buckets
    };
    // println!("buckets buckets: {:?}",buckets);

    let percentage_buckets = {
        let mut percentage_buckets = [Default::default(); DISTRIBUTION_BUCKETS];
        for i in 0..DISTRIBUTION_BUCKETS {
            percentage_buckets[i] = buckets[i] as f32 / DISTRIBUTIONS_SAMPLES as f32;
        }
        percentage_buckets
    };
    // println!("buckets percentage_buckets: {:.3?}",percentage_buckets);

    percentage_buckets
}

/// Gets a gradient of `N` colours between `start`and `end`.
fn gradient<const N: usize>(start: [f32; 4], end: [f32; 4]) -> [[f32; 4]; N] {
    let step = [
        (end[0] - start[0]) / N as f32,
        (end[1] - start[1]) / N as f32,
        (end[2] - start[2]) / N as f32,
        (end[3] - start[3]) / N as f32,
    ];
    let mut current = start;
    let mut colours = [[Default::default(); 4]; N];
    for i in 0..N {
        colours[i] = current;

        current[0] += step[0];
        current[1] += step[1];
        current[2] += step[2];
        current[3] += step[3];
    }
    colours
}
// 0 -> 0,
// 0.5 -> GRADIENT_DETAIL/2,
// 1 -> GRADIENT_DETAIL
fn get_gradient_colour(x: f32) -> [f32; 4] {
    let index = ((GRADIENT_DETAIL - 1) as f32 * x) as usize;
    debug_assert!(index <= GRADIENT_DETAIL, "{}", index);
    GRADIENT[((GRADIENT_DETAIL - 1) as f32 * x) as usize]
}
