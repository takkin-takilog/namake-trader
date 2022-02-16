# The price data (open, high, low, close) for the Candlestick.
# Reference:
#    https://developer.oanda.com/rest-live-v20/instrument-df/#CandlestickData

# Ask candle(OHLC) data.
float32 ask_o
float32 ask_h
float32 ask_l
float32 ask_c

# Bid candle(OHLC) data.
float32 bid_o
float32 bid_h
float32 bid_l
float32 bid_c

# Start time of the candlestick.
# Format is "%Y-%m-%dT%H:%M:%S"
string time

# A flag indicating if the latest candlestick is complete.
bool is_complete
