import ssl
import time

import adafruit_bmp280
import adafruit_minimqtt.adafruit_minimqtt as mqtt_class
import adafruit_sht31d
import adafruit_tmp117
import analogio
import board
import busio
import pwmio
import socketpool
import wifi
from adafruit_htu21d import HTU21D
from adafruit_motor import servo
from digitalio import DigitalInOut, Direction

try:
  from secrets import secrets
except ImportError:
  print( "WiFi secrets are kept in secrets.py, please add them there!" )
  raise

print( f"Connecting to {secrets['ssid']}" )
wifi.radio.connect( secrets['ssid'], secrets['password'] )
print( f"Connected to {secrets['ssid']}" )

# MQTT Topic
mqtt_topic = "PicoW"
mqtt_command_topic = mqtt_topic + "commands"

led = DigitalInOut( board.LED )
led.direction = Direction.OUTPUT
led.value = True


def blink( times ):
  for _ in range( times ):
    led.value = False
    time.sleep( 0.1 )
    led.value = True
    time.sleep( 0.1 )


# Create I2C bus objects
stemma_i2c = busio.I2C( board.GP5, board.GP4 )
# i2c = busio.I2C( board.GP9, board.GP8 )
# Create sensor objects, communicating over the board's default I2C bus
tmp117 = True
if tmp117 is not None:
  tmp117_sensor = adafruit_tmp117.TMP117( stemma_i2c )
  tmp117_temp = [21.12, 21.12, 21.12]
htu = None
if htu is not None:
  htu21d_sensor = HTU21D( stemma_i2c )
  htu_temp = [21.12, 21.12, 21.12]
  htu_humidity = [21.12, 21.12, 21.12]
bmp = None
if bmp is not None:
  bmp280_sensor = adafruit_bmp280.Adafruit_BMP280_I2C( stemma_i2c, address = 0x76 )
  bmp280_sensor.sea_level_pressure = 1013.25
  bmp_temp = [21.12, 21.12, 21.12]
  bmp_pressure = [1000.1, 1000.1, 1000.1]
tmp235 = True
if tmp235 is not None:
  # Create TMP36 analog input.
  tmp235_temp = [21.12, 21.12, 21.12]
  tmp235 = analogio.AnalogIn( board.GP28 )
sht30 = True
if sht30 is not None:
  sht_temp = [21.12, 21.12, 21.12]
  sht_humidity = [21.12, 21.12, 21.12]
  sht30_sensor = adafruit_sht31d.SHT31D( stemma_i2c )


# Define callback methods which are called when events occur
# pylint: disable=unused-argument, redefined-outer-name
def connect( connect_client, userdata, flags, rc ):
  # This function will be called when the mqtt_client is connected
  # successfully to the broker.
  print( "Connected to MQTT Broker!" )
  print( f"  Flags: {flags}" )
  print( f"  RC: {rc}" )
  if userdata is not None:
    print( f"  Client: {connect_client}" )
    print( f"  User data: {userdata}" )


def disconnect( disconnect_client, userdata, rc ):
  # This method is called when the mqtt_client disconnects
  # from the broker.
  print( "Disconnected from MQTT Broker!" )
  print( f"  RC: {rc}" )
  if userdata is not None:
    print( f"  Client: {disconnect_client}" )
    print( f"  User data: {userdata}" )


def subscribe( subscribe_client, userdata, topic, granted_qos ):
  # This method is called when the mqtt_client subscribes to a new feed.
  print( f"Subscribed to {topic} with QOS level {granted_qos}" )
  if userdata is not None:
    print( f"  Client: {subscribe_client}" )
    print( f"  User data: {userdata}" )


def unsubscribe( unsubscribe_client, userdata, topic, pid ):
  # This method is called when the mqtt_client unsubscribes from a feed.
  print( f"Unsubscribed from {topic} with PID {pid}" )
  if userdata is not None:
    print( f"  Client: {unsubscribe_client}" )
    print( f"  User data: {userdata}" )


def publish( publish_client, userdata, topic, pid ):
  # This method is called when the mqtt_client publishes data to a feed.
  print( f"Published to {topic}" )
  if userdata is not None:
    print( f"  Client: {publish_client}" )
    print( f"  User data: {userdata}" )
    print( f"  PID {pid}" )


def message( message_client, topic, on_message ):
  # Method called when a client's subscribed feed has a new value.
  print( f"Client {message_client} received a message on topic '{topic}'" )
  print( f"{on_message}" )


def mqtt_connect():
  # Create a socket pool
  pool = socketpool.SocketPool( wifi.radio )

  # Set up a MiniMQTT Client
  client = mqtt_class.MQTT(
    broker = secrets['broker'],
    port = secrets['port'],
    socket_pool = pool,
    ssl_context = ssl.create_default_context(),
  )

  # Connect callback handlers to client
  client.on_connect = connect
  client.on_disconnect = disconnect
  client.on_subscribe = subscribe
  client.on_unsubscribe = unsubscribe
  client.on_publish = publish
  client.on_message = message

  print( f"Attempting to connect to {client.broker}" )
  client.connect()

  print( f"Subscribing to {mqtt_command_topic}" )
  client.subscribe( mqtt_command_topic )
  return client


def servo_setup():
  pwm_servo = pwmio.PWMOut( board.GP0, duty_cycle = 2 ** 15, frequency = 50 )
  servo1 = servo.Servo( pwm_servo, min_pulse = 500, max_pulse = 2200 )  # tune pulse for specific servo
  return servo1


# Servo test
def servo_normal_test( servo ):
  sleep_time = 10
  angle = 0
  print( f"servo test: {angle}" )
  servo.angle = angle
  time.sleep( sleep_time )
  angle = 180
  print( f"servo test: {angle}" )
  servo.angle = angle
  time.sleep( sleep_time )
  angle = 0
  print( f"servo test: {angle}" )
  servo.angle = angle
  time.sleep( sleep_time )
  angle = 180
  print( f"servo test: {angle}" )
  servo.angle = angle
  time.sleep( sleep_time )


def add_value( input_list, value ):
  """
  This will copy element 1 to position 2,
  move element 0 to position 1,
  and add the value to element 0
  :param input_list: the list to add a value to
  :param value: the value to add
  """
  if len( input_list ) == 3:
    input_list[2] = input_list[1]
    input_list[1] = input_list[0]
    input_list[0] = value


def average_list( input_list ):
  """
  This will calculate the average of all numbers in a List
  :param input_list: the List to average
  :return: the average of all values in the List
  """
  return sum( input_list ) / len( input_list )


# Function to simplify the math of reading the temperature.
def tmp235_temperature_c( analog_in ):
  cur_value = analog_in.value
  print( f"  ADC raw value: {cur_value}" )
  ref_voltage = analog_in.reference_voltage
  print( f"  ADC reference voltage: {ref_voltage}" )
  milli_volts = cur_value * (ref_voltage * 1000 / 65535)
  print( f"  ADC reference millivolts: {milli_volts}" )
  return (milli_volts - 500) / 10


def poll_telemetry():
  print( f"Polling all sensors..." )
  if htu is not None:
    add_value( htu_temp, htu21d_sensor.temperature )
    add_value( htu_humidity, htu21d_sensor.relative_humidity )
  if tmp117 is not None:
    add_value( tmp117_temp, tmp117_sensor.temperature )
  if bmp is not None:
    add_value( bmp_temp, bmp280_sensor.temperature )
    add_value( bmp_pressure, bmp280_sensor.pressure )
  if tmp235 is not None:
    add_value( tmp235_temp, tmp235_temperature_c( tmp235 ) )
  if sht30 is not None:
    add_value( sht_temp, sht30_sensor.temperature )
    add_value( sht_humidity, sht30_sensor.relative_humidity )


def c_to_f( value ):
  return value * 1.8 + 32


def m_to_feet( value ):
  return value * 3.28084


def hpa_to_inhg( value ):
  return value / 33.863886666667


def print_telemetry():
  if htu is not None:
    print( f"HTU21D humidity: {average_list( htu_humidity ):.2f} %" )
    print( f"HTU21D temperature: {average_list( htu_temp ):.2f} C, {c_to_f( average_list( htu_temp ) ):.2f} F" )
  if tmp117 is not None:
    print( f"TMP117 temperature: {average_list( tmp117_temp ):.2f} C, {c_to_f( average_list( tmp117_temp ) ):.2f} F" )
  if bmp is not None:
    print( f"BMP280 temperature: {average_list( bmp_temp ):.2f} C, {c_to_f( average_list( bmp_temp ) ):.2f} F" )
    print( f"BMP280 pressure: {average_list( bmp_pressure ):.2f} hPa, {hpa_to_inhg( average_list( bmp_pressure ) ):.2f} inHg" )
    print( f"BMP280 altitude: {bmp280_sensor.altitude:.2f} meters, {m_to_feet( bmp280_sensor.altitude ):.2f} feet" )
  if tmp235 is not None:
    print( f"TMP235 temperature: {average_list( tmp235_temp ):.2f} C, {c_to_f( average_list( tmp235_temp ) ):.2f} F" )
  if sht30 is not None:
    print( f"SHT30 humidity: {average_list( sht_humidity ):.2f} %" )
    print( f"SHT30 temperature: {average_list( sht_temp ):.2f} C, {c_to_f( average_list( sht_temp ) ):.2f} F" )


def publish_telemetry( root_topic ):
  if htu is not None:
    mqtt_client.publish( f"{root_topic + '/htu21d/tempC'}", average_list( htu_temp ) )
    mqtt_client.publish( f"{root_topic + '/htu21d/tempF'}", c_to_f( average_list( htu_temp ) ) )
    mqtt_client.publish( f"{root_topic + '/htu21d/humidity'}", average_list( htu_humidity ) )
  if tmp117 is not None:
    mqtt_client.publish( f"{root_topic + '/tmp117/tempC'}", average_list( tmp117_temp ) )
    mqtt_client.publish( f"{root_topic + '/tmp117/tempF'}", c_to_f( average_list( tmp117_temp ) ) )
  if bmp is not None:
    mqtt_client.publish( f"{root_topic + '/bmp280/tempC'}", average_list( bmp_temp ) )
    mqtt_client.publish( f"{root_topic + '/bmp280/tempF'}", c_to_f( average_list( bmp_temp ) ) )
    mqtt_client.publish( f"{root_topic + '/bmp280/pressure'}", average_list( bmp_pressure ) )
    mqtt_client.publish( f"{root_topic + '/bmp280/altitude'}", bmp280_sensor.altitude )
  if tmp235 is not None:
    mqtt_client.publish( f"{root_topic + '/tmp235/tempC'}", average_list( tmp235_temp ) )
    mqtt_client.publish( f"{root_topic + '/tmp235/tempF'}", c_to_f( average_list( tmp235_temp ) ) )
  if sht30 is not None:
    mqtt_client.publish( f"{root_topic + '/sht30/tempC'}", average_list( sht_temp ) )
    mqtt_client.publish( f"{root_topic + '/sht30/tempF'}", c_to_f( average_list( sht_temp ) ) )
    mqtt_client.publish( f"{root_topic + '/sht30/humidity'}", average_list( sht_humidity ) )


def infinite_loop():
  sensor_interval = 15  # Seconds
  last_sensor_poll = 0
  try:
    while True:
      if (time.time() - last_sensor_poll) > sensor_interval:
        print()
        poll_telemetry()
        print_telemetry()
        publish_telemetry( mqtt_topic )
        print()
        print()
        last_sensor_poll = time.time()
  finally:
    print( f"Unsubscribing from {mqtt_command_topic}" )
    mqtt_client.unsubscribe( mqtt_command_topic )

    print( f"Disconnecting from {mqtt_client.broker}" )
    mqtt_client.disconnect()


if __name__ == "__main__":
  mqtt_client = mqtt_connect()
  # Connect to GPIO 28, so the other two ADCs (26 and 27) can be used for I2C if needed.
  # tmp235_input = machine.ADC( 28 )

  # Pre-populate the lists
  poll_telemetry()
  poll_telemetry()

  infinite_loop()
