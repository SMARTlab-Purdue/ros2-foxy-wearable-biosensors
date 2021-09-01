Header header
#Breathing Rate msg
# rate: zephyr summary -> respiration_rate
float32  rate

# confidence: zephyr summary -> bool(device_worn_confidence)
bool  confidence

# reliable: zephyr summary -> not(respiration_rate_unreliable)
bool   reliable

