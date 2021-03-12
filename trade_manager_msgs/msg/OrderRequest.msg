# Limit order request Definitions.

# Order Type ID definition
int32 ORDER_TYP_MARKET=1	# Market Order
int32 ORDER_TYP_LIMIT=2		# Limit Order
int32 ORDER_TYP_STOP=3		# Stop Order

# Order Direction ID definition
int32 DIR_LONG=1	# Long order
int32 DIR_SHORT=2	# Short Order

# The Instrument.
trade_manager_msgs/Instrument inst_msg

# The order type(market or limit or stop).
int32 order_type

# The order direction(long or short).
int32 order_dir

# The quantity requested to be filled.
# The value must be "0" or positive.
# If a negative value is set, the request will be invalid.
# If "0" is set, be requested maximum order quantity in current account assets.
int32 units

# The entry price.
# If the order type is "MARKET", this parameter is ignored.
float32 entry_price

# The entry expiration time.
# The format is "%Y-%m-%dT%H:%M:%S".
# If the order type is "MARKET", this parameter is ignored.
# If is set an empty string(""), indefinite period.
string entry_exp_time

# The exit order price of take profit.
float32 take_profit_price

# The exit order price of stop loss.
float32 stop_loss_price

# The exit expiration time.
# The format is "%Y-%m-%dT%H:%M:%S".
# If is set an empty string(""), indefinite period.
string exit_exp_time
