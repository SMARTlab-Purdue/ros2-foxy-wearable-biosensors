Header header
#Heartrate msg
# rate: zephyr summary -> heart_rate
uint8  rate

# confidence: zephyr summary -> heart_rate_confidence
uint8  confidence

# reliable: zephyr summary -> not(heart_rate_unreliable)
bool   reliable

