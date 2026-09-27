# sim_ground_truth_viz

Gazebo ground truth of the tables and objects for RViz, to compare any
perception result (boxes, point clouds, scene graphs) with the truth in the
same view. Simulation only; on a real robot it publishes nothing.

```bash
roslaunch sim_ground_truth_viz sim_ground_truth_viz.launch   # demo_sim.launch starts it (start_ground_truth_viz:=true)
```

| Topic | Shows |
| --- | --- |
| `/sim_ground_truth/boxes` | oriented box per model (blue objects, grey tables) around its URDF collision primitives, label `GT <model>` |
| `/sim_ground_truth/meshes` | the model's URDF visuals: textured meshes, or the primitives a configurable model is made of (a table: top + legs) |

Both are `visualization_msgs/MarkerArray` in `map` (`frame_id` arg) and appear
in the *Ground truth (sim)* group of `tables_demo_bringup/config/pick_n_place.rviz`.

- **URDFs** come from the parameter server, where the spawn launch files put
  them: `/<model>_description`, `/<label>_description` (label = model name
  without `_<n>`) or, for models spawned from a shared parameter such as the
  tables, the per-world map in `config/models.yaml` (the world is detected from
  its `pbr_<world>` model). Models without a URDF are a small sphere labelled
  `(no size)`.
- **Boxes** are the bounding box of the collision boxes, cylinders and spheres;
  models that collide with a mesh take theirs from `box_overrides` in
  `config/models.yaml`.
- **Lazy**: `/gazebo/model_states` is subscribed only while one of the topics
  has a subscriber, and a topic is republished (at most 1 Hz) only when a model
  moved more than 5 mm.
