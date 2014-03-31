#Enumerated action types
uint16 STEP=0
uint16 VIEW=1

#Enumerated node_ids

#major id is for chronological ordering later
uint32 major_id

#action type tells which subplanner should handle this move
uint16 action_type

#which frame to move/target, and where to go
string node_name
float32 x
float32 y
float32 theta