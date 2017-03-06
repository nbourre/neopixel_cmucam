#include "SPI.h"
#include <Pixy.h>
#include <Adafruit_NeoMatrix.h>
//#include <Adafruit_NeoPixel.h>

#ifdef __AVR__
  #include <avr/power.h>
#endif

#define NEOPIX_PIN  3
#define NUMPIXELS   64
#define MAT_WIDTH   8;
#define MAT_HEiGHT  8;

#define NEOPIX_DELAY 50

#define X_CENTER        ((PIXY_MAX_X-PIXY_MIN_X)/2)       
#define Y_CENTER        ((PIXY_MAX_Y-PIXY_MIN_Y)/2)

Adafruit_NeoMatrix matrix = Adafruit_NeoMatrix (8, 8, 3,
  NEO_MATRIX_BOTTOM + NEO_MATRIX_LEFT +
  NEO_MATRIX_COLUMNS + NEO_MATRIX_PROGRESSIVE,
  NEO_GRB + NEO_KHZ800);

Pixy pixy;

long currentTime = millis();
long previousTime = 0;
int delta = 0;

int matrix_acc = 0;

const uint16_t colors[] = {
  matrix.Color(255, 0, 0), matrix.Color(0, 255, 0), matrix.Color(0, 0, 255) };

int objectWidth;

class ServoLoop
{
public:
  ServoLoop(int32_t pgain, int32_t dgain);

  void update(int32_t error);
   
  int32_t m_pos;
  int32_t m_prevError;
  int32_t m_pgain;
  int32_t m_dgain;
};


ServoLoop panLoop(300, 500);
ServoLoop tiltLoop(500, 700);

ServoLoop::ServoLoop(int32_t pgain, int32_t dgain)
{
  m_pos = PIXY_RCS_CENTER_POS;
  m_pgain = pgain;
  m_dgain = dgain;
  m_prevError = 0x80000000L;
}

void ServoLoop::update(int32_t error)
{
  long int vel;
  char buf[32];
  if (m_prevError!=0x80000000)
  {  
    vel = (error*m_pgain + (error - m_prevError)*m_dgain)>>10;
    //sprintf(buf, "%ld\n", vel);
    //Serial.print(buf);
    m_pos += vel;
    if (m_pos>PIXY_RCS_MAX_POS) 
      m_pos = PIXY_RCS_MAX_POS; 
    else if (m_pos<PIXY_RCS_MIN_POS) 
      m_pos = PIXY_RCS_MIN_POS;
  }
  m_prevError = error;
}


void setup() {
  Serial.begin(9600);
  Serial.print("Starting...\n");
  
  pixy.init();
  // put your setup code here, to run once:
  matrix.begin();
  matrix.setBrightness(40);

}

void loop() {
  currentTime = millis();
  delta = currentTime - previousTime;
  
  pixyLoop();
  matrixLoop (delta);

  previousTime = currentTime;
  
}

void matrixLoop(int delta) {
  matrix_acc += delta;
  if (matrix_acc > NEOPIX_DELAY) {
    matrix_acc = 0;

    int x = (int)((float)(panLoop.m_pos) / (float)PIXY_RCS_MAX_POS * 7);
    int y = (int)((float)(tiltLoop.m_pos) / (float)PIXY_RCS_MAX_POS * 7);
    
    matrix.fillScreen(0);

    matrix.drawPixel(x, y, colors[1]);
    int w = (float)objectWidth / (float)160 * MAT_WIDTH;

    if (w % 2 == 0 && w > 1) {
      w = w - 1;
    }

    matrix.drawRect(x - w / 2, y - w / 2, w, w, colors[2]);
    
    matrix.show();
  }

}

void pixyLoop () {
  static int i = 0;
  int j;
  uint16_t blocks;
  char buf[32]; 
  int32_t panError, tiltError;
  
  blocks = pixy.getBlocks();
  
  if (blocks)
  {
    panError = X_CENTER-pixy.blocks[0].x;
    tiltError = pixy.blocks[0].y-Y_CENTER;
    
    panLoop.update(panError);
    tiltLoop.update(tiltError);

  
    pixy.setServos(panLoop.m_pos, tiltLoop.m_pos);

    objectWidth = pixy.blocks[j].width;
    
    i++;
    
    // do this (print) every 50 frames because printing every
    // frame would bog down the Arduino
    if (i%50==0) 
    {
      sprintf(buf, "Detected %d:\n", blocks);
      Serial.print(buf);
      for (j=0; j<blocks; j++)
      {

        sprintf(buf, "  block %d: ", j);
        Serial.print(buf); 
        pixy.blocks[j].print();

        
        int w = pixy.blocks[j].width;
        int h = pixy.blocks[j].height;
        int dist = getDistance(65, 17, 300, w, h);

        
        
        sprintf(buf, "\tDistance : %d mm ", dist);
        Serial.println(buf);
      }
    }
  }  
}

/**
 * Return the distance in mm of a tag
 * params :
 *  tagWidth, tagHeight : Tag dimensions in mm
 *  refWidthPx, refHeightPx : Reference in pixels at given dimensions
 *  refDistance : Reference distance at given dimensions and pixels
 *  mesWidthPx, mesHeightPx : Measured tag in pixels
 */
int getDistance (int refWidthPx, int refHeightPx, int refDistance, int mesWidthPx, int mesHeightPx) {
  float ratioW = ((float)refWidthPx / mesWidthPx + (float)refHeightPx / mesHeightPx) / 2;
  return ratioW * refDistance;  
}



