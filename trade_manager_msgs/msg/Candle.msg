# The price data (open, high, low, close) for the Candlestick for monitor.

# The ask candle data.
float32 ask_o
float32 ask_h
float32 ask_l
float32 ask_c

# The bid candle data.
float32 bid_o
float32 bid_h
float32 bid_l
float32 bid_c

# The mid candle data.
float32 mid_o
float32 mid_h
float32 mid_l
float32 mid_c

# The mid candle data.
float32 half_spread_cost_o
float32 half_spread_cost_h
float32 half_spread_cost_l
float32 half_spread_cost_c

# The start time of the candlestick.
# Format is "%Y-%m-%dT%H:%M:00.000000000Z"
string time

# A flag indicating if the latest candlestick is complete.
bool is_complete
