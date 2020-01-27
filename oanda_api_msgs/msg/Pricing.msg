# Pricing Definitions of OANDA-API.
# Reference:
#    https://developer.oanda.com/rest-live-v20/pricing-df/
#    https://oanda-api-v20.readthedocs.io/en/latest/endpoints/pricing/pricingstream.html

# The date/time when the Price was created
string time

# The Price’s Instrument.
string instrument

# The closeout bid Price.
float32 closeout_bid

# The closeout ask Price.
float32 closeout_ask

# Flag indicating if the Price is tradeable or not
bool tradeable
