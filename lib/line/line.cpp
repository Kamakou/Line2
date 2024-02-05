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

void Line::LineAlgo(){//50行は多すぎ？
      uint8_t white_count=WhiteNum();
      static int16_t prv_inside_degree=0;
      if (white_count<=2){
            outside_=halfout_;
            dir_inside=outside_?prv_inside_degree:-1;
      }else{
            outside_=false;
            uint8_t pos_w[white_count];
            uint8_t num_w=0;
            for (uint8_t i=0;i<LINE_QTY;i ++){
                  if(val[i] > th_val[i]){
                        pos_w[num_w]=i;
                        num_w ++;
                  }
            }
            uint8_t intv_line[white_count];
            for (uint8_t i=0;i<white_count-1;i++){
                  intv_line[i]=pos_w[i+1]-pos_w[i];
            }
            intv_line[white_count-1]=pos_w[0]-pos_w[white_count]+LINE_QTY;
            uint8_t max_interval=0;
            uint8_t pos_maxinterval=0;
            for (num_w=0;num_w<white_count-1;num_w++){
                  if(max_interval<intv_line[num_w]){
                        max_interval=intv_line[num_w];
                        pos_maxinterval=num_w;
                  }
            }
            dir_inside=pos_w[pos_maxinterval]+max_interval*0.5;
            inside_vector_x=MyCos(dir_inside*360/LINE_QTY);
            inside_vector_y=MySin(dir_inside*360/LINE_QTY);
            inside_degree=atan2(inside_vector_y,inside_vector_x)*180/PI;
            if (prv_inside_degree>=0){
                  halfout_=false;
                  if(prv_inside_degree+110<inside_degree and inside_degree<prv_inside_degree+250){
                        inside_degree=inside_degree+180;
                        halfout_=true;
                  }
            }
      }
      prv_inside_degree=inside_degree;
}//inside_degreeを実際の動く向きに設定、prv_inside_degreeに一つ前の値が保持されているか確認する必要あり

uint8_t Line::IsOutside(){
      return outside_;
}

uint8_t Line::IsHalfout(){
      return halfout_;
}

int16_t Line::MovingVec(){
      return inside_degree;
}

int16_t Line::DirInside(){
      return dir_inside;
}



