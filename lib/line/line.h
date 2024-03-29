#ifndef MBED_LINE_H
#define MBED_LINE_H

#include "approximate_atan2.h"
#include "mbed.h"
#include "simplify_deg.h"
#include "sin_cos_table.h"

#define PI 3.1415926535   // 円周率

#define LINE_QTY 24
#define SET_TH_NUM 100
#define TH_VALUE 10
#define FULL_COUNT 1      //仮

class Line {
     public:
      Line(PinName left_, PinName right_,
           PinName mux1a_, PinName mux1b_, PinName mux1x_, PinName mux1y_,
           PinName mux2a_, PinName mux2b_, PinName mux2x_, PinName mux2y_,
           PinName mux3a_, PinName mux3b_, PinName mux3x_, PinName mux3y_);

      void Read();
      void SetTh();
      void LineAlgo();
      uint8_t IsLeft();
      uint8_t IsRight();
      uint8_t WhiteNum();
      int16_t LineVector();
      uint8_t IsOutside();
      uint8_t IsHalfout();
      int16_t MovingVec();
      int16_t DirInside();

     private:
      AnalogIn left;
      AnalogIn right;
      DigitalOut mux1a;
      DigitalOut mux1b;
      AnalogIn mux1x;
      AnalogIn mux1y;
      DigitalOut mux2a;
      DigitalOut mux2b;
      AnalogIn mux2x;
      AnalogIn mux2y;
      DigitalOut mux3a;
      DigitalOut mux3b;
      AnalogIn mux3x;
      AnalogIn mux3y;

      uint8_t val[LINE_QTY];
      uint16_t th_val[LINE_QTY];
      bool is_white[LINE_QTY];

      uint8_t left_val;
      uint8_t right_val;
      uint16_t left_th_val;
      uint16_t right_th_val;
      bool is_left_white;
      bool is_right_white;

      float unit_vector_x[LINE_QTY];
      float unit_vector_y[LINE_QTY];
      float result_vector_x = 0;
      float result_vector_y = 0;
      bool outside_= false;
      bool halfout_= false;
      int16_t inside_degree=0;
      int16_t dir_inside=0;
      float inside_vector_x=0;
      float inside_vector_y=0;
};

#endif