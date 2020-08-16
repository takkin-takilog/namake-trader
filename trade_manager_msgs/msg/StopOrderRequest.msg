# Stop order request Definitions.

# instrument ID definition
int32 INST_USD_JPY=1	# USD/JPY
int32 INST_EUR_JPY=2	# EUR/JPY
int32 INST_EUR_USD=3	# EUR/USD

# The request ID.
int32 req_id

# The instrument ID.
int32 instrument_id

# The number of units to open order for.
# If positive value is set, it is a "buy" direction,
# and if a negative value is set, it is a "sell" direction.
int32 units

# The "limit" or "stop" order price.
float32 price

# The valid period of the new order.
# If want to set indefinite period, set blank as "".
string valid_period_new

# The price of settlement order (take profit).
float32 take_profit_price

# The price of settlement order (stop loss).
float32 stop_loss_price

# The valid period of the settlement order.
# If want to set indefinite period, set blank as "".
string valid_period_settlement
