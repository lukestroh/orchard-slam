# Orchard.msg
# Metadata information
uint64 orchard_seed # needed for generating unique orchards given the same parameters
string orchard_name
string tree_namespace
string tree_type

# orchard generation parameters
int64 n_rows
int64 total_num_trees
float32 avg_trees_per_row 
float32 trees_per_row_std
float32 avg_tree_spacing # average spacing between trees in a row, in meters
float32 tree_spacing_std
float32 avg_row_deviation # how much the trees in a row deviate from a straight line, in meters
float32 std_row_deviation
float32 avg_row_spacing # average spacing between rows, in meters
float32 row_spacing_std


orchard_msgs/OrchardRow[] rows

uint64 seed # unique seed given orchard parameters and orchard_seed