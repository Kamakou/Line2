#include "line.h"

#include "mbed.h"
Line::Line(PinName left_, PinName right_, PinName mux1a_, PinName mux1b_, PinName mux1x_, PinName mux1y_, PinName mux2a_, PinName mux2b_, PinName mux2x_, PinName mux2y_, PinName mux3a_, PinName mux3b_, PinName mux3x_, PinName mux3y_)
    : left(left_), right(right_), mux1a(mux1a_), mux1b(mux1b_), mux1x(mux1x_), mux1y(mux1y_), mux2a(mux2a_), mux2b(mux2b_), mux2x(mux2x_), mux2y(mux2y_), mux3a(mux3a_), mux3b(mux3b_), mux3x(mux3x_), mux3y(mux3y_) {
      mux1a = 0;
      mux1b = 0;
      mux2a = 0;
      mux2b = 0;
      mux3a = 0;
      mux3b = 0;

      for (uint8_t i = 0; i < LINE_QTY; i++) {   // それぞれのセンサにベクトルを与える
            unit_vector_x[i] = MyCos(i * 360.00000 / LINE_QTY);
            unit_vector_y[i] = MySin(i * 360.00000 / LINE_QTY);
      }
}

void Line::Read() {
      left_val = left.read_u16() / 256;
      right_val = right.read_u16() / 256;

      mux1a = 0;
      mux1b = 0;
      mux2a = 0;
      mux2b = 0;
      mux3a = 0;
      mux3b = 0;
      wait_us(5);
      val[22] = mux1x.read_u16() / 256;
      val[19] = mux1y.read_u16() / 256;
      val[15] = mux2x.read_u16() / 256;
      val[11] = mux2y.read_u16() / 256;
      val[2] = mux3x.read_u16() / 256;
      val[5] = mux3y.read_u16() / 256;

      mux1a = 1;
      mux1b = 0;
      mux2a = 1;
      mux2b = 0;
      mux3a = 1;
      mux3b = 0;
      wait_us(5);
      val[21] = mux1x.read_u16() / 256;
      val[16] = mux1y.read_u16() / 256;
      val[14] = mux2x.read_u16() / 256;
      val[8] = mux2y.read_u16() / 256;
      val[3] = mux3x.read_u16() / 256;
      val[0] = mux3y.read_u16() / 256;

      mux1a = 0;
      mux1b = 1;
      mux2a = 0;
      mux2b = 1;
      mux3a = 0;
      mux3b = 1;
      wait_us(5);
      val[20] = mux1x.read_u16() / 256;
      val[18] = mux1y.read_u16() / 256;
      val[13] = mux2x.read_u16() / 256;
      val[10] = mux2y.read_u16() / 256;
      val[6] = mux3x.read_u16() / 256;
      val[4] = mux3y.read_u16() / 256;

      mux1a = 1;
      mux1b = 1;
      mux2a = 1;
      mux2b = 1;
      mux3a = 1;
      mux3b = 1;
      wait_us(5);
      val[23] = mux1x.read_u16() / 256;
      val[17] = mux1y.read_u16() / 256;
      val[12] = mux2x.read_u16() / 256;
      val[9] = mux2y.read_u16() / 256;
      val[7] = mux3x.read_u16() / 256;
      val[1] = mux3y.read_u16() / 256;

      if (left_val > left_th_val) {
            is_left_white = 1;
      } else {
            is_left_white = 0;
      }
      if (right_val > right_th_val) {
            is_right_white = 1;
      } else {
            is_right_white = 0;
      }
      for (uint16_t i = 0; i < LINE_QTY; i++) {
            if (val[i] > th_val[i]) {
                  is_white[i] = 1;
            } else {
                  is_white[i] = 0;
            }
      }
}

void Line::SetTh() {
      left_th_val = 0;
      right_th_val = 0;
      for (uint16_t i = 0; i < LINE_QTY; i++) {
            th_val[i] = 0;
      }

      for (uint16_t i = 0; i < SET_TH_NUM; i++) {
            Read();
            left_th_val += left_val;
            right_th_val += right_val;

            for (uint16_t j = 0; j < LINE_QTY; j++) {
                  th_val[j] += val[j];
            }
      }

      left_th_val /= SET_TH_NUM;
      right_th_val /= SET_TH_NUM;
      left_th_val += TH_VALUE;
      right_th_val += TH_VALUE;
      for (uint16_t i = 0; i < LINE_QTY; i++) {
            th_val[i] /= SET_TH_NUM;
            th_val[i] += TH_VALUE;
      }
}

int16_t Line::LineVector() {
      result_vector_x = 0;
      result_vector_y = 0;
      int16_t degree;

      for (uint8_t i = 0; i < LINE_QTY; i++) {   // 全てのセンサのベクトルを合成
            result_vector_x += is_white[i] * unit_vector_x[i];
            result_vector_y += is_white[i] * unit_vector_y[i];
      }

      degree = atan2(result_vector_y, result_vector_x) * 180.00 / PI;
      return degree;
}

uint8_t Line::IsLeft() {
      return is_left_white;
}

uint8_t Line::IsRight() {
      return is_right_white;
}

uint8_t Line::WhiteNum() {
      uint8_t white_sum;
      for (uint8_t i = 0; i < LINE_QTY; i++) {   // 全てのセンサのベクトルを合成
            white_sum += is_white[i];
      }
      return white_sum;
}

int8_t Line::JUDGE(){
      int8_t Line_judge;
      if (WhiteNum() < 2){
            Line_judge=0;//ライン上でない
      }else{
            Line_judge=1;//ライン上
      }
      return Line_judge;
}

uint8_t Line::cprDegree(){
      int16_t degree_Before=LineVector();
      static int16_t degree_After=0;
      uint8_t degree_cpr;
      if (degree_After>degree_Before){//角度増加
            degree_cpr=2;
      }else if(degree_After<degree_Before){//角度減少
            degree_cpr=1;
      }else{
            degree_cpr=0;//不変
      }
      degree_After=degree_Before;
      return degree_cpr;
}

float Line::Memory_vecx(){
      static int16_t Memory_x;
      int8_t JUDGE_A=JUDGE();
      static int8_t JUDGE_B;
      int8_t JUDGE_res=JUDGE_A-JUDGE_B;
      if(JUDGE_res=1){
            Memory_x=result_vector_x;//xベクトル
      }else if(JUDGE_res=-1){//ライン内に戻ったため記憶をリセット
            Memory_x=0;
      }else{
            //何もしない
      }
      JUDGE_B=JUDGE_A;
      return Memory_x;
}

float Line::Memory_vecy(){
      static int16_t Memory_y;
      int8_t JUDGE_A=JUDGE();
      static int8_t JUDGE_B;
      int8_t JUDGE_res=JUDGE_A-JUDGE_B;
      if(JUDGE_res=1){
            Memory_y=result_vector_y;//xベクトル
      }else if(JUDGE_res=-1){//ライン内に戻ったため記憶をリセット
            Memory_y=0;
      }else{
            //何もしない
      }
      JUDGE_B=JUDGE_A;
      return Memory_y;
}

float Line::Moving_x(){
      float Moving_vecx;
      if (result_vector_x >=0)&&(Memory_vecx()<=0){
            Moving_vecx = -1*(result_vector_x);
      }else if(result_vector_x<=0)&&(Memory_vecx()>=0){
            Moving_vecx = -1*(result_vector_x);
      }else {
            Moving_vecx=1*(result_vector_x);
      }
      return Moving_vecx;
}

float Line::Moving_y(){
      float Moving_vecy;
      if (result_vector_y >=0)&&(Memory_vecy()<=0){
            Moving_vecy = -1*(result_vector_y);
      }else if(result_vector_y<=0)&&(Memory_vecy()>=0){
            Moving_vecy = -1*(result_vector_y);
      }else {
            Moving_vecy=1*(result_vector_y);
      }
      return Moving_vecy;
}
