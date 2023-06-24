#include "esp_timer.h"
#include "esp_camera.h"
#include "camera_index.h"
#include "Arduino.h"
#include "fd_forward.h"
#include "fr_forward.h"

#define CAMERA_MODEL_AI_THINKER
#include "camera_pins.h"

camera_fb_t * fb = NULL;

typedef struct
{
  uint8_t *image;
  box_array_t *net_boxes; 
} img_process_result;

static inline mtmn_config_t app_mtmn_config()
{
  mtmn_config_t mtmn_config = {0};
  mtmn_config.type = FAST;
  mtmn_config.min_face = 80;
  mtmn_config.pyramid = 0.707;
  mtmn_config.pyramid_times = 4;
  mtmn_config.p_threshold.score = 0.6;
  mtmn_config.p_threshold.nms = 0.7;
  mtmn_config.p_threshold.candidate_number = 20;
  mtmn_config.r_threshold.score = 0.7;
  mtmn_config.r_threshold.nms = 0.7;
  mtmn_config.r_threshold.candidate_number = 10;
  mtmn_config.o_threshold.score = 0.7;
  mtmn_config.o_threshold.nms = 0.7;
  mtmn_config.o_threshold.candidate_number = 1;
  return mtmn_config;
}
mtmn_config_t mtmn_config = app_mtmn_config();

dl_matrix3du_t *image_matrix; 
img_process_result out_res = {0};  

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  //init with high specs to pre-allocate larger buffers
  if (psramFound()) {
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }
  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_QVGA);

  // THIS SIZE IS DIRECTLY RELATED WITH FRAMESIZE_QVGA = (320,240)!!!!!!!!!!!
  // TO CHANGE IT, CHANGE THE VALUE ON config.frame_size AND HERE.
  image_matrix = dl_matrix3du_alloc(1, 320, 240, 3); 
  out_res.image = image_matrix->item;
}

void loop() {
  fb = esp_camera_fb_get(); // THIS IS THE FRAME BUFFER (A POINTER TO)
  out_res.net_boxes = NULL; // INTEREST POINTS (BOXES)

  /*
    The structure:
    out_res.net_boxes -----> box_array_t

    typedef struct tag_box_list
  {
      box_t *box;
      landmark_t *landmark; ------------> The points!!
      int len; -----------> How many?
  } box_array_t;
  */

  fmt2rgb888(fb->buf, fb->len, fb->format, out_res.image); // rgb888 NEEDED TO WORK WITH face_detect 
  out_res.net_boxes = face_detect(image_matrix, &mtmn_config);

  if (out_res.net_boxes){
    int x, y, w, h, i;
    Serial.print("image_matrix->w ");
    Serial.println(image_matrix->w);
    Serial.print("image_matrix->h ");
    Serial.println(image_matrix->h);
    for (int i = 0; i < out_res.net_boxes->len; i++){ // FOR EVERY BOX
      // rectangle box
      x = (int)out_res.net_boxes->box[i].box_p[0];
      y = (int)out_res.net_boxes->box[i].box_p[1];
      w = (int)out_res.net_boxes->box[i].box_p[2] - x + 1;
      h = (int)out_res.net_boxes->box[i].box_p[3] - y + 1;
    }
    // The values!
    Serial.print("Box ");
    Serial.print(i + 1);
    Serial.print(": x = ");
    Serial.print(x);
    Serial.print(", y = ");
    Serial.print(y);
    Serial.print(", width = ");
    Serial.print(w);
    Serial.print(", height = ");
    Serial.println(h);
    Serial.println();
  }
  esp_camera_fb_return(fb);
  fb = NULL;
}
