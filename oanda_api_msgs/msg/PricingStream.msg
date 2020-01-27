# Pricing Definitions of OANDA-API.
# Reference:
#    https://developer.oanda.com/rest-live-v20/pricing-df/
#    https://oanda-api-v20.readthedocs.io/en/latest/endpoints/pricing/pricingstream.html

# The date/time when the Price was created
string time

# The Price’s Instrument.
string instrument

# prices available on the Instrument’s bid side.
float32 bid_price

# prices available on the Instrument’s ask side.
float32 ask_price

# Flag indicating if the Price is tradeable or not
bool tradeable