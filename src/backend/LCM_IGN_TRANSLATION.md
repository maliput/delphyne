# LCM ignition translation

## DRAKE_VIEWER_LOAD_ROBOT mapping

Drake publishes in the `DRAKE_VIEWER_LOAD_ROBOT` channel the model of the robot that should be loaded by the visualizer. This channel uses the `lcmt_viewer_load_robot` type, which transitively spawns to the following types:

```
struct lcmt_viewer_load_robot {
  int32_t num_links;
  lcmt_viewer_link_data link[num_links];
}

struct lcmt_viewer_link_data {
  string name;
  int32_t robot_num;

  int32_t num_geom;
  lcmt_viewer_geometry_data geom[num_geom];
}

struct lcmt_viewer_geometry_data {
  int8_t type;

  // Defines an enum for geometry type.
  const int8_t BOX          = 1;
  const int8_t SPHERE       = 2;
  const int8_t CYLINDER     = 3;
  const int8_t MESH         = 4;
  const int8_t CAPSULE      = 5;
  const int8_t ELLIPSOID    = 6;

  float position[3];
  float quaternion[4];
  float color[4];

  string string_data;

  int32_t num_float_data;
  float float_data[num_float_data];
}
```

with this in mind, we can apply the following one-way mapping between LCM messages and ignition ones:

### General

| LCM                                               | ign-msgs                                      |
|:--------------------------------------------------|----------------------------------------------:|
| viewer_load_robot_t.link[n].geom[k].type          | Model.link[n].visual[k].geometry.type         |
| viewer_load_robot_t.link[n].geom[k].BOX = 1       | Model.link[n].visual[k].geometry.BOX = 1      |
| viewer_load_robot_t.link[n].geom[k].SPHERE = 2    | Model.link[n].visual[k].geometry.SPHERE = 3   |
| viewer_load_robot_t.link[n].geom[k].CYLINDER = 3  | Model.link[n].visual[k].geometry.CYLINDER = 2 |
| viewer_load_robot_t.link[n].geom[k].MESH = 4      | Model.link[n].visual[k].geometry.MESH = 7     |
| viewer_load_robot_t.link[n].geom[k].position[0]   | Model.link[n].visual[k].pose.position.x       |
| viewer_load_robot_t.link[n].geom[k].position[1]   | Model.link[n].visual[k].pose.position.y       |
| viewer_load_robot_t.link[n].geom[k].position[2]   | Model.link[n].visual[k].pose.position.z       |
| viewer_load_robot_t.link[n].geom[k].quaternion[0] | Model.link[n].visual[k].pose.orientation.x    |
| viewer_load_robot_t.link[n].geom[k].quaternion[1] | Model.link[n].visual[k].pose.orientation.y    |
| viewer_load_robot_t.link[n].geom[k].quaternion[2] | Model.link[n].visual[k].pose.orientation.z    |
| viewer_load_robot_t.link[n].geom[k].quaternion[3] | Model.link[n].visual[k].pose.orientation.w    |
| viewer_load_robot_t.link[n].geom[k].color[0]      | Model.link[n].visual[k].material.diffuse.r    |
| viewer_load_robot_t.link[n].geom[k].color[1]      | Model.link[n].visual[k].material.diffuse.g    |
| viewer_load_robot_t.link[n].geom[k].color[2]      | Model.link[n].visual[k].material.diffuse.b    |
| viewer_load_robot_t.link[n].geom[k].color[3]      | Model.link[n].visual[k].material.diffuse.a    |

## Box

| LCM                                               | ign-msgs                                    |
|:--------------------------------------------------|--------------------------------------------:|
| viewer_load_robot_t.link[n].geom[k].float_data[0] | Model.link[n].visual[k].geometry.box.size.x |
| viewer_load_robot_t.link[n].geom[k].float_data[1] | Model.link[n].visual[k].geometry.box.size.y |
| viewer_load_robot_t.link[n].geom[k].float_data[2] | Model.link[n].visual[k].geometry.box.size.z |

## Cylinder

| LCM                                               | ign-msgs                                         |
|:--------------------------------------------------|-------------------------------------------------:|
| viewer_load_robot_t.link[n].geom[k].float_data[0] | Model.link[n].visual[k].geometry.cylinder.radius |
| viewer_load_robot_t.link[n].geom[k].float_data[1] | Model.link[n].visual[k].geometry.cylinder.length |

## Mesh from file

| LCM                                              | ign-msgs                                       |
|:-------------------------------------------------|-----------------------------------------------:|
| viewer_load_robot_t.link[n].geom[k].string_data  | Model.link[n].visual[k].geometry.mesh.filename |
| viewer_load_robot_t.link[n].geom[k].float_data[0]| Model.link[n].visual[k].geometry.mesh.scale.x  |
| viewer_load_robot_t.link[n].geom[k].float_data[1]| Model.link[n].visual[k].geometry.mesh.scale.y  |
| viewer_load_robot_t.link[n].geom[k].float_data[2]| Model.link[n].visual[k].geometry.mesh.scale.z  |

## Mesh from mesh-array

| LCM                                             | ign-msgs                                       |
|:------------------------------------------------|-----------------------------------------------:|
| viewer_load_robot_t.link[n].geom[k].float_data  | Mesh-array doesn't have an ignition counterpart|

## Sphere

| LCM                                               | ign-msgs                                       |
|:--------------------------------------------------|-----------------------------------------------:|
| viewer_load_robot_t.link[n].geom[k].float_data[0] | Model.link[n].visual[k].geometry.sphere.radius |

## Unmatched geometries

| LCM                                                    | ign-msgs                       |
|:-------------------------------------------------------|-------------------------------:|
| viewer_load_robot_t.link[n].geom[k].type.CAPSULE = 5   |                                |
| viewer_load_robot_t.link[n].geom[k].type.ELLIPSOID = 6 |                                |

**Important:** Note that the `float_data` array holds different values depending on the geometry type (e.g. it has 3 elements when describing a box -x, y and z- while two elements when describing a cylinder -radius and length-).

## DRAKE_VIEWER_DRAW mapping

Drake publishes in the `DRAKE_VIEWER_DRAW` channel all the links of each of the robots loaded by the visualizer. In contrast with `DRAKE_VIEWER_LOAD_ROBOT`, this message is sent continuously across all the simulation's lifespan, updating the link poses and orientations. This channel uses the `lcmt_viewer_draw` type, with the following definition:

```
struct lcmt_viewer_draw {
  // The timestamp in milliseconds.
  int64_t timestamp;

  int32_t num_links;
  string link_name[num_links];
  int32_t robot_num[num_links];
  float position[num_links][3];
  float quaternion[num_links][4];
}
```
with this in mind, we can apply the following one-way mapping between LCM messages and ignition ones:


| LCM                            | ign-msgs                           |
|:-------------------------------|-----------------------------------:|
| viewer_draw_t.timestamp        | PosesStamped.time                  |
| viewer_draw_t.robot_num[i]     | PosesStamped.pose[i].id            |
| viewer_draw_t.link_name[i]     | PosesStamped.pose[i].name          |
| viewer_draw_t.position[i][0]   | PosesStamped.pose[i].position.x    |
| viewer_draw_t.position[i][1]   | PosesStamped.pose[i].position.y    |
| viewer_draw_t.position[i][2]   | PosesStamped.pose[i].position.z    |
| viewer_draw_t.quaternion[i][0] | PosesStamped.pose[i].orientation.x |
| viewer_draw_t.quaternion[i][1] | PosesStamped.pose[i].orientation.y |
| viewer_draw_t.quaternion[i][2] | PosesStamped.pose[i].orientation.z |
| viewer_draw_t.quaternion[i][3] | PosesStamped.pose[i].orientation.w |
