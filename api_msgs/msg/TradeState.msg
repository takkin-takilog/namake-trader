# Trade State Definitions.
# Reference:
#    https://developer.oanda.com/rest-live-v20/trade-df/#TradeState

# Order type definition
int32 STS_OPEN=1
int32 STS_CLOSED=2
int32 STS_CLOSE_WHEN_TRADEABLE=3

# The current state of the Trade.
int32 state
