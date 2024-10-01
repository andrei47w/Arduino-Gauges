#include <Adafruit_GC9A01A.h>
#include <LinkedList.h>

//  int n - The number to be displayed
//  int xLoc = The x location of the upper left corner of the number
//  int yLoc = The y location of the upper left corner of the number
//  int cSe = The size of the number. Range 1 to 10 uses Large Shaped Segments.
//  fC is the foreground color of the number
//  bC is the background color of the number (prevents having to clear previous
//  space) nD is the number of digit spaces to occupy (must include space for
//  minus sign for numbers < 0) nD < 0 Suppresses leading zero
void draw7FloatingNumber(Adafruit_GC9A01A &tft, float n, unsigned int xLoc,
                         unsigned int yLoc, char cS, unsigned int fC,
                         unsigned int bC, char nD, unsigned int precision) {
  unsigned int S2 = 5 * cS;  // width of horizontal segments   5 times the cS
  unsigned int S3 = 2 * cS;  // thickness of a segment 2 times the cs
  unsigned int S4 = 5 * cS;  // height of vertical segments 7 times the cS
  unsigned int x1 = cS + 1;  // starting x location of horizontal segments
  unsigned int x2 = S3 + S2 + 1;  // starting x location of right side segments

  int dot_size = S3;

  int intiger = n;
  int floating = abs(n - int(n)) * pow(10, precision);
  char tmp_nD = nD;
  unsigned int tmp_xLoc = xLoc;

  draw7Number(tft, n, tmp_xLoc, yLoc, cS, fC, bC, tmp_nD);

  // if intiget is 0, force it to draw the 0
  if (intiger == 0) {
    tmp_xLoc += (S2 + 3 * S3 + 2) * (abs(nD) - 1);
    tmp_nD = 1;
    draw7Number(tft, intiger, tmp_xLoc, yLoc, cS, fC, bC, tmp_nD);
  }

  if (precision != 0) {
    int xDot = xLoc + (2 * S3 + S2) * abs(nD) + 4 * dot_size;
    int yDot = yLoc + (2 * S3 + S4) * 2 - dot_size - cS;
    tft.fillRect(xDot, yDot, dot_size, dot_size, fC);

    xLoc = xDot + 2 * dot_size;
    draw7Number(tft, floating, xLoc, yLoc, cS, fC, bC, precision);
  }
}

void draw7Number(Adafruit_GC9A01A &tft, int n, unsigned int xLoc,
                 unsigned int yLoc, char cS, unsigned int fC, unsigned int bC,
                 char nD) {
  unsigned int num = abs(n), i, s, t, w, col, h, a, b, si = 0, j = 1, d = 0;
  unsigned int S2 = 5 * cS;  // width of horizontal segments   5 times the cS
  unsigned int S3 = 2 * cS;  // thickness of a segment 2 times the cs
  unsigned int S4 = 5 * cS;  // height of vertical segments 7 times the cS
  unsigned int x1 = cS + 1;  // starting x location of horizontal segments
  unsigned int x2 = S3 + S2 + 1;  // starting x location of right side segments
  unsigned int y1 = yLoc + x1;    // starting y location of top side segments
  unsigned int y3 =
      yLoc + S3 + S4 + 1;  // starting y location of bottom side segments
  unsigned int seg[7][3] = {{x1, yLoc, 1},    {x2, y1, 0},
                            {x2, y3 + x1, 0}, {x1, (2 * y3) - yLoc, 1},
                            {0, y3 + x1, 0},  {0, y1, 0},
                            {x1, y3, 1}};  // actual x,y locations of all 7
                                           // segments with direction
  unsigned char nums[12] = {
      0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D,
      0x07, 0x7F, 0x67, 0x00, 0x40};  // segment defintions for all 10 numbers
                                      // plus blank and minus sign
  unsigned char c, cnt;

  c = abs(cS);  // get character size between 1 and 10 ignoring sign
  if (c > 10) c = 10;
  if (c < 1) c = 1;

  cnt = abs(nD);  // get number of digits between 1 and 10 ignoring sign
  if (cnt > 10) cnt = 10;
  if (cnt < 1) cnt = 1;

  xLoc += (cnt - 1) *
          (d = S2 + (3 * S3) + 2);  // set starting x at last digit location

  while (cnt > 0) {  // for cnt number of places

    --cnt;

    if (num > 9)
      i = num % 10;  // get the last digit
    else if (!cnt && n < 0)
      i = 11;  // show minus sign if 1st position and negative number
    else if (nD < 0 && !num)
      i = 10;  // show blanks if remaining number is zero
    else
      i = num;

    num = num / 10;  // trim this digit from the number

    for (j = 0; j < 7; ++j) {  // draw all seven segments

      if (nums[i] & (1 << j))
        col = fC;  // if segment is On use foreground color
      else
        col = bC;  // else use background color

      if (seg[j][2]) {
        w = S2;                     // Starting width of segment (side)
        t = seg[j][1] + S3;         // maximum thickness of segment
        h = seg[j][1] + cS;         // half way point thickness of segment
        a = xLoc + seg[j][0] + cS;  // starting x location
        b = seg[j][1];              // starting y location

        while (b < h) {                     // until x location = half way
          tft.drawFastHLine(a, b, w, col);  // Draw a horizontal segment top
          a--;                              // move the x position by -1
          b++;                              // move the y position by 1
          w += 2;                           // make the line wider by 2
        }

      } else {
        w = S4;                     // Starting height of segment (side)
        t = xLoc + seg[j][0] + S3;  // maximum thickness of segment
        h = xLoc + seg[j][0] + cS;  // half way point thickness of segment
        a = seg[j][1] + cS;         // starting y location
        b = xLoc + seg[j][0];       // starting x location

        while (b < h) {                     // until x location = half way
          tft.drawFastVLine(b, a, w, col);  // Draw a vertical line right side
          a--;                              //  move the y position by -1
          b++;                              //  move teh x position by 1
          w += 2;                           //  make the line wider by 2
        }
      }

      while (b < t) {  // finish drawing horizontal bottom or vertical left side
                       // of segment
        if (seg[j][2]) {
          tft.drawFastHLine(a, b, w, col);  // Draw Horizonal line bottom
        } else {
          tft.drawFastVLine(b, a, w, col);  // Draw Vertical line left side
        }
        b++;  // keep moving the x or y draw position until t
        a++;  // move the length or height starting position back the other way.
        w -= 2;  // move the length or height back the other way
      }
    }

    xLoc -= d;  // move to next digit position
  }
}

/*
D11 -> TFT1 SDA
D7 -> TFT3 CS
D6 -> TFT2 CS
D5 -> TFT1 CS
D2 -> TFT1 DC

D13 -> TFT1 SCL
3V3 -> TFT1 VCC
A2 -> 2.5K RESISTOR
A3 -> BLU SENSOR CABLE (PRESS)
A4 -> ADXL SDA
A5 -> ADL SCL
5V -> POWER LINE
GND -> TFT1 GND
GND -> GND LINE
*/

#define DELAY 50

// Define pin connections for each display
#define TFT_CS1 6
#define TFT_CS2 5
#define TFT_CS3 7
#define TFT_DC 2

// Define analog pin for the temperature sensor
#define WATER_TEMP_SENSOR_PIN A7
#define OIL_TEMP_SENSOR_PIN A2
#define PRESS_SENSOR_PIN A1
#define SCREEN_SWITCH_PIN A0
#define PRESS_SWITCH_PIN 10

#define SENSOR_VOLTAGE 5

#define COLOR 0xf800
#define COLOR_GOOD 0x26e4
#define COLOR_GOOD_FAINT 0x0140
#define COLOR_BAD 0xfe17
#define COLOR_MEH 0xfe00
#define BACKGROUND 0x0000
#define PLOT_COLOR COLOR_GOOD

#define START_ANGLE 45
#define END_ANGLE 315

#define PLOT_SIZE 120
#define ACCEL_SIZE 40

#define PLOT_X 60
#define PLOT_Y 167
#define PLOT_HEIGHT 50.
#define PLOT_UPDATE_MS 15000  // plot  ais should be 10 min
#define AVG_SIZE 3            // 150 ms

const float p1 = 0.000000000001753474230565890000;
const float p2 = 0.000000002072841370044680000000;
const float p3 = 0.000000987993765003445000000000;
const float p4 = 0.000244989747178747000000000000;
const float p5 = 0.034323277948919900000000000000;
const float p6 = 2.918681602988160000000000000000;
const float p7 = 200.2342258266050000000000000000;

const float n1 = 0.000000000000004899676590175890;
const float n2 = 0.000000000017287182540416100000;
const float n3 = 0.000000024926920046616900000000;
const float n4 = 0.000018620384231904400000000000;
const float n5 = 0.007511992737848760000000000000;
const float n6 = 1.429195318009230000000000000000;
const float n7 = 48.90444428767570000000000000000;

int16_t TEMP_MIN = -40;
uint8_t TEMP_MAX = 140;
uint8_t TEMP_MIN_GOOD = 75;
uint8_t TEMP_MAX_GOOD = 105;
uint8_t TEMP_DIGITS = 3;

uint8_t PRESS_MIN = 0;
uint8_t PRESS_MAX = 10;
float PRESS_MIN_GOOD = 0.7;
uint8_t PRESS_MAX_GOOD = 7;
uint8_t PRESS_DIGITS = 2;

float last_watertemp_number = -100;
float avg_watertemp = 0;
float last_watertemp_circle = TEMP_MIN;
LinkedList<uint16_t> plotWaterTemp = LinkedList<uint16_t>();

float last_oiltemp_number = -100;
float avg_oiltemp = 0;
float last_oiltemp_circle = TEMP_MIN;
LinkedList<uint16_t> plotOilTemp = LinkedList<uint16_t>();

float last_oilpressure_number = -100;
float avg_oilpressure = 0;
float last_oilpressure_circle = PRESS_MIN;
LinkedList<uint16_t> plotOilPress = LinkedList<uint16_t>();

int16_t avg_x = 0;
int16_t avg_y = 0;
uint8_t current_avg_size = 0;
// int8_t accelList[ACCEL_SIZE];
int16_t last_x = 0;
int16_t last_y = 0;
int16_t x_max = 0;
int16_t x_min = 0;
int16_t y_max = 0;
int16_t y_min = 0;

float TEMP_DEGREE_UNIT = (TEMP_MAX - TEMP_MIN) / 360.;
float PRESS_DEGREE_UNIT = (PRESS_MAX - PRESS_MIN) / 360.;

bool flash_oil_temp = 0;
bool flash_oil_press = 0;
bool flash_water_temp = 0;
bool switch_is_on = 1;
bool button_is_on = analogRead(SCREEN_SWITCH_PIN) >= 900;

// Create instances for each display
Adafruit_GC9A01A tft1 = Adafruit_GC9A01A(TFT_CS1, TFT_DC, -1);
Adafruit_GC9A01A tft2 = Adafruit_GC9A01A(TFT_CS2, TFT_DC, -1);
Adafruit_GC9A01A tft3 = Adafruit_GC9A01A(TFT_CS3, TFT_DC, -1);

Adafruit_I2CDevice *i2c_dev = NULL;

unsigned long startTime;
bool doReplot = true;

void setup() {
  pinMode(PRESS_SWITCH_PIN, OUTPUT);

  tft1.begin();
  tft2.begin();
  tft3.begin();

  tft1.setRotation(2);
  tft1.fillScreen(BACKGROUND);

  tft2.setRotation(2);
  tft2.fillScreen(BACKGROUND);

  tft3.setRotation(2);
  tft3.fillScreen(BACKGROUND);

  tft1.setTextColor(PLOT_COLOR);
  tft1.setTextSize(2);
  tft1.setCursor(105, 40);
  tft1.print(F("oil"));

  tft2.setTextColor(PLOT_COLOR);
  tft2.setTextSize(2);
  tft2.setCursor(90, 40);
  tft2.print(F("press"));

  tft3.setTextColor(PLOT_COLOR);
  tft3.setTextSize(2);
  tft3.setCursor(90, 40);
  tft3.print(F("water"));

  Serial.begin(9600);
  begin();

  startupAnimation();

  startTime = millis();
}

void loop() {
  // check plot update interval
  unsigned long loopStart = millis();
  if (loopStart - startTime >= PLOT_UPDATE_MS) {
    doReplot = true;
    startTime = loopStart;
  }

  // OIL TEMPERATURE
  int o = analogRead(OIL_TEMP_SENSOR_PIN);
  float oiltemp = 0;

  if (o <= 325) {
    oiltemp = p1 * pow(o, 6) - p2 * pow(o, 5) + p3 * pow(o, 4) -
              p4 * pow(o, 3) + p5 * pow(o, 2) - p6 * o + p7;
  } else {
    oiltemp = -n1 * pow(o, 6) + n2 * pow(o, 5) - n3 * pow(o, 4) +
              n4 * pow(o, 3) - n5 * pow(o, 2) + n6 * o - n7;
  }
  oiltemp -= 2.5;
  if (avg_oiltemp == 0) {
    avg_oiltemp = oiltemp;
  } else {
    avg_oiltemp = (avg_oiltemp + oiltemp) / 2;
  }

  if (abs(oiltemp - last_oiltemp_number) > 0.1 || oiltemp >= TEMP_MAX_GOOD) {
    updateDisplay(tft1, oiltemp, last_oiltemp_number, 1, TEMP_MIN_GOOD,
                  TEMP_MAX_GOOD, 1, TEMP_DIGITS, &flash_oil_temp);
    last_oiltemp_number = oiltemp;
  }
  if (abs(oiltemp - last_oiltemp_circle) > TEMP_DEGREE_UNIT) {
    updateCircle(tft1, oiltemp, last_oiltemp_circle, TEMP_MIN, TEMP_MAX,
                 TEMP_MIN_GOOD, TEMP_MAX_GOOD, 1, 0);
    last_oiltemp_circle = oiltemp;
  }
  if (doReplot) {
    updatePlot(tft1, plotOilTemp, avg_oiltemp, 0);
    avg_oiltemp = oiltemp;
  }

  // CHECK FOR BUTTON PRESS
  bool is_pressed = analogRead(SCREEN_SWITCH_PIN) >= 1020;

  if (is_pressed != button_is_on) {
    tft3.fillScreen(BACKGROUND);
    button_is_on = is_pressed;

    if (is_pressed) {
      // reset accel
      x_max = 0;
      x_min = 0;
      y_max = 0;
      y_min = 0;

      tft3.drawFastVLine(120, 35, 170, COLOR_GOOD_FAINT);
      tft3.drawFastHLine(50, 120, 140, COLOR_GOOD_FAINT);

      // clear temp
      plotWaterTemp.clear();
    } else {
      // reset temp
      tft3.setTextColor(PLOT_COLOR);
      tft3.setTextSize(2);
      tft3.setCursor(90, 40);
      tft3.print(F("water"));
      updateCircle(tft3, TEMP_MAX, TEMP_MIN, TEMP_MIN, TEMP_MAX, TEMP_MIN_GOOD,
                   TEMP_MAX_GOOD, 1, 1);
      last_watertemp_number = -100;
      last_watertemp_circle = TEMP_MIN;
      avg_watertemp = 0;

      // force update plots
      startTime -= PLOT_UPDATE_MS;
      clearPlotPixels(tft1);
      plotOilPress.clear();
      clearPlotPixels(tft2);
      plotOilTemp.clear();
      clearPlotPixels(tft3);
      plotWaterTemp.clear();
    }
  }

  if (!button_is_on) {
    // WATER TEMPERATURE
    int w = analogRead(WATER_TEMP_SENSOR_PIN);
    float watertemp = 0;

    if (w <= 325) {
      watertemp = p1 * pow(w, 6) - p2 * pow(w, 5) + p3 * pow(w, 4) -
                  p4 * pow(w, 3) + p5 * pow(w, 2) - p6 * w + p7;
    } else {
      watertemp = -n1 * pow(w, 6) + n2 * pow(w, 5) - n3 * pow(w, 4) +
                  n4 * pow(w, 3) - n5 * pow(w, 2) + n6 * w - n7;
    }
    watertemp -= 1.5;
    if (avg_watertemp == 0) {
      avg_watertemp = watertemp;
    } else {
      avg_watertemp = (avg_watertemp + watertemp) / 2;
    }

    if (abs(watertemp - last_watertemp_number) > 0.1 ||
        watertemp >= TEMP_MAX_GOOD) {
      updateDisplay(tft3, watertemp, last_watertemp_number, 1, TEMP_MIN_GOOD,
                    TEMP_MAX_GOOD, 1, TEMP_DIGITS, &flash_water_temp);
      last_watertemp_number = watertemp;
    }
    if (abs(watertemp - last_watertemp_circle) > TEMP_DEGREE_UNIT) {
      updateCircle(tft3, watertemp, last_watertemp_circle, TEMP_MIN, TEMP_MAX,
                   TEMP_MIN_GOOD, TEMP_MAX_GOOD, 1, 0);
      last_watertemp_circle = watertemp;
    }
    if (doReplot) {
      updatePlot(tft3, plotWaterTemp, avg_watertemp, 0);
      avg_watertemp = watertemp;
    }
  } else {
    // ACCEL

    int16_t x = read16(0x32) * 3.99719054;
    int16_t y = read16(0x34) * 3.99719054;

    if (AVG_SIZE > current_avg_size) {
      avg_x += x / AVG_SIZE;
      avg_y += y / AVG_SIZE;
      current_avg_size++;
    } else {
      current_avg_size = 0;
      avg_x = x;
      avg_y = y;

      updateAccelDisplay(tft3, x, y);
    }
  }

  // PRESSURE
  float p = analogRead(PRESS_SENSOR_PIN) * (SENSOR_VOLTAGE / 1023.0);
  float oilpressure = ((p / SENSOR_VOLTAGE - 0.1) / 0.0008) / 100;
  if (avg_oilpressure == 0) {
    avg_oilpressure = oilpressure;
  } else {
    avg_oilpressure = (avg_oilpressure + oilpressure) / 2;
  }

  updateOilSwitch(oilpressure, &switch_is_on);
  if (abs(oilpressure - last_oilpressure_number) > 0.01 ||
      oilpressure <= PRESS_MIN_GOOD) {
    updateDisplay(tft2, oilpressure, last_oilpressure_number, 2, PRESS_MIN_GOOD,
                  PRESS_MAX_GOOD, 0, PRESS_DIGITS, &flash_oil_press);
    last_oilpressure_number = oilpressure;
  }
  if (abs(oilpressure - last_oilpressure_circle) > PRESS_DEGREE_UNIT) {
    updateCircle(tft2, oilpressure, last_oilpressure_circle, PRESS_MIN,
                 PRESS_MAX, PRESS_MIN_GOOD, PRESS_MAX_GOOD, 0, 0);
    last_oilpressure_circle = oilpressure;
  }
  if (doReplot) {
    updatePlot(tft2, plotOilPress, max(avg_oilpressure, 0), 0);
    avg_oilpressure = oilpressure;
  }

  unsigned long loopTime = millis() - loopStart;
  if (loopTime < DELAY) {
    delay(DELAY - loopTime);
  }

  if (doReplot) {
    doReplot = false;
  }
}

int16_t read16(uint8_t reg) {
  uint8_t buffer[2] = {reg};
  if (i2c_dev) {
    i2c_dev->write(buffer, 1);
    i2c_dev->read(buffer, 2);
  }

  return uint16_t(buffer[1]) << 8 | uint16_t(buffer[0]);
}

bool begin() {
  uint8_t i2caddr = 0x53;
  if (i2c_dev) delete i2c_dev;

  i2c_dev = new Adafruit_I2CDevice(i2caddr, &Wire);

  if (!i2c_dev->begin()) return false;

  /* Check connection */
  uint8_t buffer_id[1] = {0x00};
  if (i2c_dev) {
    i2c_dev->write(buffer_id, 1);
    i2c_dev->read(buffer_id, 1);
  }
  uint8_t deviceid = buffer_id[0];

  if (deviceid != 0xE5) {
    /* No ADXL345 detected ... return false */
    return false;
  }

  // Enable measurements
  uint8_t buffer[2] = {0x2D, 0x08};
  if (i2c_dev) {
    i2c_dev->write(buffer, 2);
  }

  return true;
}

void updateCircle(Adafruit_GC9A01A &tft, float value, float lastValue,
                  float min, float max, float good_min, float good_max,
                  bool meh_direction, bool isStartupAnimation) {
  value = max(min(value, max), min);
  lastValue = max(min(lastValue, max), min);

  int startAngle =
      (max - value) / (max - min) * (END_ANGLE - START_ANGLE) + START_ANGLE;
  int endAngle =
      (max - lastValue) / (max - min) * (END_ANGLE - START_ANGLE) + START_ANGLE;
  int goodMinAngle =
      (max - good_min) / (max - min) * (END_ANGLE - START_ANGLE) + START_ANGLE;
  int goodMaxAngle =
      (max - good_max) / (max - min) * (END_ANGLE - START_ANGLE) + START_ANGLE;
  unsigned int width = isStartupAnimation ? 2 : 15;
  float offsetRad = -1.57;

  unsigned int color;
  if (endAngle > startAngle) {
    color = COLOR;
    startAngle -= 3;
  } else {
    color = BACKGROUND;
    unsigned int aux = startAngle;
    startAngle = endAngle - 3;
    endAngle = aux;
  }

  unsigned int centerX = 120;
  unsigned int centerY = 120;
  unsigned int outerRadius = isStartupAnimation ? 105 : 120;
  unsigned int innerRadius = outerRadius - width;
  float goodMinRad = goodMinAngle * M_PI / 180.0 + offsetRad;
  float goodMaxRad = goodMaxAngle * M_PI / 180.0 + offsetRad;

  // Loop through angles from startAngle to endAngle
  for (unsigned int angleDeg = endAngle; angleDeg >= startAngle; angleDeg--) {
    // Convert degree to radians (angleDeg * pi / 180)
    float angleRad = angleDeg * M_PI / 180.0 + offsetRad;

    // Precompute sin and cos values once per angle
    float cosAngle = cos(angleRad);
    float sinAngle = sin(angleRad);

    // Compute outer and inner boundary points
    unsigned int outerX = centerX + outerRadius * cosAngle;
    unsigned int outerY = centerY - outerRadius * sinAngle;

    unsigned int innerX = centerX + innerRadius * cosAngle;
    unsigned int innerY = centerY - innerRadius * sinAngle;

    // Draw a line between the inner and outer radius to create the width
    if (color != BACKGROUND) {
      if (angleRad < goodMaxRad) {
        color = !meh_direction ? COLOR_MEH : COLOR;
      } else if (angleRad < goodMinRad) {
        color = COLOR_GOOD;
      } else {
        color = meh_direction ? COLOR_MEH : COLOR;
      }
    }

    tft.drawLine(innerX, innerY, outerX, outerY, color);
  }
}

void updateOilSwitch(float value, bool *switch_is_on) {
  bool on = value <= PRESS_MIN_GOOD;

  if (on != switch_is_on) {
    digitalWrite(PRESS_SWITCH_PIN, on ? HIGH : LOW);
    *switch_is_on = on;
  }
}

// Function to update the display with the new temperature
void updateDisplay(Adafruit_GC9A01A &tft, float value, float lastValue,
                   int precision, float good_min, float good_max,
                   bool meh_direction, int digitCount, bool *last_flash) {
  uint8_t x = 35;
  uint8_t y = 80;
  uint8_t size = 3;

  unsigned int color;
  if (value < good_min) {
    if (meh_direction) {
      color = COLOR_MEH;
    } else {
      color = *last_flash ? COLOR : COLOR_BAD;
      *last_flash = !*last_flash;
    }
  } else if (value < good_max) {
    color = COLOR_GOOD;
  } else {
    if (!meh_direction) {
      color = COLOR_MEH;
    } else {
      color = *last_flash ? COLOR : COLOR_BAD;
      *last_flash = !*last_flash;
    }
  }

  draw7FloatingNumber(tft, value, x, y, size, color, BACKGROUND, -digitCount,
                      precision);
}

void updateAccelDisplay(Adafruit_GC9A01A &tft, int16_t x, int16_t y) {
  uint8_t dot_size = 5;

  // draw current accel
  tft.fillCircle(last_x / 18 + 120, last_y / 18 + 120, dot_size,
                 COLOR_GOOD_FAINT);
  tft.fillCircle(x / 18 + 120, y / 18 + 120, dot_size, COLOR);
  last_x = x;
  last_y = y;

  // draw text
  tft.setTextSize(2);

  if (x_min > x || x_min == 0) {
    tft.setTextColor(BACKGROUND);
    tft.setCursor(2, 113);
    tft.print(abs((float)x_min / 1000));
    x_min = min(x_min, x);

    tft.setTextColor(COLOR_GOOD);
    tft.setCursor(2, 113);
    tft.print(abs((float)x_min / 1000), 1);
  }
  if (x_max < x || x_max == 0) {
    tft.setTextColor(BACKGROUND);
    tft.setCursor(200, 113);
    tft.print(abs((float)x_max / 1000), 1);
    x_max = max(x_max, x);

    tft.setTextColor(COLOR_GOOD);
    tft.setCursor(200, 113);
    tft.print(abs((float)x_max / 1000), 1);
  }
  if (y_min > y || y_min == 0) {
    tft.setTextColor(BACKGROUND);
    tft.setCursor(102, 7);
    tft.print(abs((float)y_min / 1000), 1);
    y_min = min(y_min, y);

    tft.setTextColor(COLOR_GOOD);
    tft.setCursor(102, 7);
    tft.print(abs((float)y_min / 1000), 1);
  }
  if (y_max < y || y_max == 0) {
    tft.setTextColor(BACKGROUND);
    tft.setCursor(102, 223);
    tft.print(abs((float)y_max / 1000), 1);
    y_max = max(y_max, y);

    tft.setTextColor(COLOR_GOOD);
    tft.setCursor(102, 223);
    tft.print(abs((float)y_max / 1000), 1);
  }
}

void clearPlotPixels(Adafruit_GC9A01A &tft) {
  tft.fillRect(PLOT_X, PLOT_Y, PLOT_SIZE, PLOT_HEIGHT + 1, BACKGROUND + 1);
}

void updatePlot(Adafruit_GC9A01A &tft, LinkedList<uint16_t> &list, float value,
                bool draw_degree) {
  uint16_t newValue = uint16_t(value * 100);
  uint16_t minValue, maxValue;
  uint8_t valuePixelWidth = 3;

  if (list.size() >= PLOT_SIZE / valuePixelWidth) {
    list.remove(0);  // Remove oldest value
  }
  list.add(newValue);
  updateMinMax(list, &minValue, &maxValue);

  // setup tft
  tft.setTextColor(PLOT_COLOR);

  // print min
  tft.setTextSize(2);
  tft.fillRect(PLOT_X + PLOT_SIZE / 4, PLOT_Y + PLOT_HEIGHT + 5, PLOT_SIZE / 2,
               20, BACKGROUND);
  tft.setCursor(PLOT_X + PLOT_SIZE / 4, PLOT_Y + PLOT_HEIGHT + 5);
  tft.print((float)minValue / 100, 1);
  if (draw_degree) {
    tft.setTextSize(1);
    tft.print(F(" o"));
  }

  // print min
  tft.setTextSize(2);
  tft.fillRect(PLOT_X + PLOT_SIZE / 4, PLOT_Y - 17, PLOT_SIZE / 2, 20,
               BACKGROUND);
  tft.setCursor(PLOT_X + PLOT_SIZE / 4, PLOT_Y - 17);
  tft.print((float)maxValue / 100, 1);
  if (draw_degree) {
    tft.setTextSize(1);
    tft.print(F(" o"));
  }

  // clear plot
  for (int i = 0; i < list.size(); i++) {
    uint16_t plotValue = list.get(i);
    uint16_t pixelValue = PLOT_HEIGHT - (maxValue - (float)plotValue) /
                                            (maxValue - minValue) * PLOT_HEIGHT;
    pixelValue = max(min(pixelValue, PLOT_HEIGHT), 1);

    // Draw a vertical line for each mapped pixel value
    int xPos = PLOT_X + i * valuePixelWidth;
    for (int j = 0; j < valuePixelWidth; j++) {
      tft.drawFastVLine(xPos, PLOT_Y + PLOT_HEIGHT, -pixelValue, PLOT_COLOR);
      tft.drawFastVLine(xPos, PLOT_Y, PLOT_HEIGHT - pixelValue, BACKGROUND);
      xPos--;
    }
  }
}

void updateMinMax(LinkedList<uint16_t> &list, uint16_t *minValue,
                  uint16_t *maxValue) {
  for (int i = 0; i < list.size(); i++) {
    uint16_t tempFloat = list.get(i);

    if (i == 0) {
      *minValue = tempFloat;
      *maxValue = tempFloat;
    } else {
      *minValue = min(tempFloat, *minValue);
      *maxValue = max(tempFloat, *maxValue);
    }
  }
}

void startupAnimation() {
  float tempValue = 0;
  float pressValue = 0;
  float tempLast = -1;
  float pressLast = -1;

  uint8_t speed = 10;

  for (tempValue = TEMP_MIN; tempValue <= TEMP_MAX; tempValue += speed) {
    pressValue = PRESS_MAX - (TEMP_MAX - tempValue) / (TEMP_MAX - TEMP_MIN) *
                                 (PRESS_MAX - PRESS_MIN);

    updateOilSwitch(pressValue, &switch_is_on);

    updateCircle(tft1, tempValue, tempLast, TEMP_MIN, TEMP_MAX, TEMP_MIN_GOOD,
                 TEMP_MAX_GOOD, 1, 1);
    updateCircle(tft3, tempValue, tempLast, TEMP_MIN, TEMP_MAX, TEMP_MIN_GOOD,
                 TEMP_MAX_GOOD, 1, 1);
    updateCircle(tft2, pressValue, pressLast, PRESS_MIN, PRESS_MAX,
                 PRESS_MIN_GOOD, PRESS_MAX_GOOD, 0, 1);

    updateDisplay(tft1, tempValue, tempLast, 1, TEMP_MIN_GOOD, TEMP_MAX_GOOD, 1,
                  TEMP_DIGITS, &flash_oil_temp);
    updateDisplay(tft3, tempValue, tempLast, 1, TEMP_MIN_GOOD, TEMP_MAX_GOOD, 1,
                  TEMP_DIGITS, &flash_water_temp);
    updateDisplay(tft2, pressValue, pressLast, 2, PRESS_MIN_GOOD,
                  PRESS_MAX_GOOD, 0, PRESS_DIGITS, &flash_oil_press);

    tempLast = tempValue;
    pressLast = pressValue;
  }
}