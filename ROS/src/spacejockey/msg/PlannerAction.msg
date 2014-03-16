#Enumerated action types
uint16 TYPE_STEP=0
uint16 TYPE_VIEW=1

#major id is for chronological ordering later
uint32 major_id

#action type tells which subplanner should handle this move
uint16 action_type

#which frame to move/target, and where to go
string node_name
float32 x
float32 y
float32 theta