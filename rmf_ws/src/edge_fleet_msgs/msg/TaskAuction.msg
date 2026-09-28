# Decentralized Contract Net Protocol (CNP) for Peer-to-Peer Task Reallocation
builtin_interfaces/Time stamp
string auction_id
string task_id

# Message types for CNP handshake:
# 1 = ANNOUNCE (Task available for bidding, e.g., due to blocked aisle)
# 2 = BID (Peer AMR submits cost bid based on distance & battery)
# 3 = AWARD (Original AMR assigns task to best bidder)
# 4 = ACK (Winning AMR confirms acceptance)
uint8 MSG_TYPE_ANNOUNCE = 1
uint8 MSG_TYPE_BID = 2
uint8 MSG_TYPE_AWARD = 3
uint8 MSG_TYPE_ACK = 4
uint8 message_type

string sender_id
string target_robot_id

geometry_msgs/Point pickup_location
geometry_msgs/Point dropoff_location

float32 bid_cost                 # Computed bid value (lower is better)
float32 sender_battery_soc       # Battery of bidder
string reason                    # Reason: "AISLE_BLOCKED", "BATTERY_CRITICAL", etc.
