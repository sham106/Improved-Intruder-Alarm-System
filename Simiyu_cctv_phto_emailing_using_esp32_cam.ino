
#include "esp_camera.h"
#include "SPI.h"
#include "driver/rtc_io.h"
#include "ESP32_MailClient.h"
#include <FS.h>
#include <SPIFFS.h>
#include <WiFi.h>
/* ======================================== */

/* ======================================== Defining variables for Email */
/*
 * Specifically for Gmail users :
 * - To send Email using Gmail use port 465 (SSL) and SMTP Server smtp.gmail.com
 * - Especially for the Sender's Gmail account, so that ESP32 CAM can log into the sender's Gmail account,
 *   the Sender's Gmail account must activate 2-Step Verification then get "App Passwords". The method is in the video. Watch carefully.
 */
#define emailSenderAccount      "douglasmuuo@gmail.com"
#define emailSenderAppPassword  "rzkrepwjykgtxxhz"
#define smtpServer              "smtp.gmail.com"
#define smtpServerPort          465
#define emailSubject            "IMPROVED INTRUDER ALARM SYSTEM"
String emailRecipient = "mobiswitch2021@gmail.com";
/* ======================================== */

/* ======================================== Defining the Camera Model and GPIO */
#define CAMERA_MODEL_AI_THINKER

#if defined(CAMERA_MODEL_AI_THINKER)
  #define PWDN_GPIO_NUM     32
  #define RESET_GPIO_NUM    -1
  #define XCLK_GPIO_NUM      0
  #define SIOD_GPIO_NUM     26
  #define SIOC_GPIO_NUM     27
  
  #define Y9_GPIO_NUM       35
  #define Y8_GPIO_NUM       34
  #define Y7_GPIO_NUM       39
  #define Y6_GPIO_NUM       36
  #define Y5_GPIO_NUM       21
  #define Y4_GPIO_NUM       19
  #define Y3_GPIO_NUM       18
  #define Y2_GPIO_NUM        5
  #define VSYNC_GPIO_NUM    25
  #define HREF_GPIO_NUM     23
  #define PCLK_GPIO_NUM     22
#else
  #error "Camera model not selected"
#endif
/* ======================================== */
 
#define FILE_PHOTO "/photo.jpg" //--> Photo File Name to save in SPIFFS
#define FLASH_GPIO_NUM 4
#define alert 15
const int buttonPin1 = 2;    // the number of the pushbutton pin
int button1State;             // the current reading from the input pin
int lastButton1State = LOW;   // the previous reading from the input pin
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;  
/* ======================================== Variables for network */
// REPLACE WITH YOUR NETWORK CREDENTIALS
const char* ssid = "Mobiswitch";
const char* password = "dimuli10";
/* ======================================== */

SMTPData smtpData; //--> The Email Sending data object contains config and data to send

/* ________________________________________________________________________________ Subroutine for Capture Photo and Save it to SPIFFS */
void capturePhotoSaveSpiffs( void ) {
  camera_fb_t * fb = NULL; //--> pointer
  bool ok = 0; //--> Boolean indicating if the picture has been taken correctly
  digitalWrite(FLASH_GPIO_NUM, HIGH);
  delay(100);
  digitalWrite(FLASH_GPIO_NUM, LOW);
  /* ---------------------------------------- The process of capturing photos and saving to SPIFFS */
  do {
    Serial.println("Taking a photo...");

    /* :::::::::::::::::::::::::::::::::::::::::::::::: Take a photo with the camera */
    fb = esp_camera_fb_get();

    if (!fb) {
      Serial.println("Camera capture failed.");
      return;
    }
    /* :::::::::::::::::::::::::::::::::::::::::::::::: */
    
    /* :::::::::::::::::::::::::::::::::::::::::::::::: Photo file name */
    Serial.printf("Picture file name: %s\n", FILE_PHOTO);
    File file = SPIFFS.open(FILE_PHOTO, FILE_WRITE);
    /* :::::::::::::::::::::::::::::::::::::::::::::::: */

    /* :::::::::::::::::::::::::::::::::::::::::::::::: Insert the data in the photo file */
    if (!file) {
      Serial.println("Failed to open file in writing mode.");
    }
    else {
      file.write(fb->buf, fb->len); // payload (image), payload length
      Serial.print("The picture has been saved in ");
      Serial.print(FILE_PHOTO);
      Serial.print(" - Size: ");
      Serial.print(file.size());
      Serial.println(" bytes.");
    }
    /* :::::::::::::::::::::::::::::::::::::::::::::::: */
    
    file.close(); //--> Close the file
    esp_camera_fb_return(fb); //--> return the frame buffer back to the driver for reuse.

    /* :::::::::::::::::::::::::::::::::::::::::::::::: check if file has been correctly saved in SPIFFS */
    Serial.println("Checking if the picture file has been saved correctly in SPIFFS...");
    ok = checkPhoto(SPIFFS);
    if (ok == 1) {
      Serial.println("The picture file has been saved correctly in SPIFFS.");
    } else {
      Serial.println("The picture file is not saved correctly in SPIFFS.");
      Serial.println("Recapture photos.");
      Serial.println();
    }
    /* :::::::::::::::::::::::::::::::::::::::::::::::: */
  } while ( !ok );
  /* ---------------------------------------- */
}
/* ________________________________________________________________________________ */

/* ________________________________________________________________________________ Function to check if photos are saved correctly in SPIFFSl */
bool checkPhoto( fs::FS &fs ) {
  File f_pic = fs.open( FILE_PHOTO );
  unsigned int pic_sz = f_pic.size();
  return ( pic_sz > 100 );
}
/* ________________________________________________________________________________ */

/* ________________________________________________________________________________ Subroutine for send photos via Email */
void sendPhoto(String emailRecipient) {
  Serial.println("Sending email...");

  // Set the SMTP Server Email host, port, account and password
  smtpData.setLogin(smtpServer, smtpServerPort, emailSenderAccount, emailSenderAppPassword); 
  
  // Set the sender name and Email
  smtpData.setSender("IMPROVED INTRUDER ALARM SYSTEM", emailSenderAccount); 
  
  // Set Email priority or importance High, Normal, Low or 1 to 5 (1 is highest)
  smtpData.setPriority("High"); 

  // Set the subject
  smtpData.setSubject(emailSubject); 

  // Set the email message in HTML format
  smtpData.setMessage("<h2>Improved Intruder Alarm System</h2>", true); 
  // Set the email message in text format
  //smtpData.setMessage("Photo captured with ESP32-CAM and attached in this email.", false); 

  // Add recipients, can add more than one recipient
  smtpData.addRecipient(emailRecipient); 
  //smtpData.addRecipient(emailRecipient2);

  // Add attach files from SPIFFS
  smtpData.addAttachFile(FILE_PHOTO, "Evidence SLTS/jpg"); 

  // Set the storage type to attach files in your email (SPIFFS)
  smtpData.setFileStorageType(MailClientStorageType::SPIFFS); 

  // sendCallback
  smtpData.setSendCallback(sendCallback); 
  
  // Start sending Email, can be set callback function to track the status
  if (!MailClient.sendMail(smtpData))
  Serial.println("Error sending Email, " + MailClient.smtpErrorReason());

  // Clear all data from Email object to free memory
  smtpData.empty();
}
/* ________________________________________________________________________________ */

/* ________________________________________________________________________________ Subroutine to get the Email sending status */
// Callback function to get the Email sending status
void sendCallback(SendStatus msg) {
  Serial.println(msg.info()); //--> Print the current status
}
/* ________________________________________________________________________________ */

/* ________________________________________________________________________________ VOID SETTUP() */
void setup() {
  // put your setup code here, to run once:
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //--> disable brownout detector
    pinMode(FLASH_GPIO_NUM, OUTPUT);
    pinMode(alert, OUTPUT);
     pinMode(buttonPin1, INPUT_PULLUP);
  Serial.begin(115200);
  delay(1000);
  Serial.println();

  /* ---------------------------------------- Connect to Wi-Fi */
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
      digitalWrite(alert,HIGH);
    delay(500);
   digitalWrite(alert,LOW);
   delay(200);
    digitalWrite(alert,HIGH);
    delay(100);
       digitalWrite(alert,LOW);
   delay(200);
    digitalWrite(alert,HIGH);
    delay(100);
       digitalWrite(alert,LOW);
   delay(1000);
    digitalWrite(alert,HIGH);
          
  Serial.println();
  Serial.print("Successfully connected to ");
  Serial.println(ssid);
  /* ---------------------------------------- */

  /* ---------------------------------------- Print ESP32 Local IP Address */
  Serial.print("IP Address: http://");
  Serial.println(WiFi.localIP());
  Serial.println();
  /* ---------------------------------------- */

  /* ---------------------------------------- Starting to mount SPIFFS */
  Serial.println("Starting to mount SPIFFS...");
  if (!SPIFFS.begin(true)) {
    Serial.println("An Error has occurred while mounting SPIFFS");
    Serial.println("ESP32 Cam Restart...");
    ESP.restart();
  }
  else {
    delay(500);
    Serial.println("SPIFFS mounted successfully");
  }
  /* ---------------------------------------- */

  /* ---------------------------------------- Camera configuration. */
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
  
  if(psramFound()){
    config.frame_size = FRAMESIZE_UXGA; //--> FRAMESIZE_ + QVGA|CIF|VGA|SVGA|XGA|SXGA|UXGA
    /*
     * From source https://randomnerdtutorials.com/esp32-cam-ov2640-camera-settings/ :
     * - The image quality (jpeg_quality) can be a number between 0 and 63.
     * - Higher numbers mean lower quality.
     * - Lower numbers mean higher quality.
     * - Very low numbers for image quality, specially at higher resolution can make the ESP32-CAM to crash or it may not be able to take the photos properly.
     */
    config.jpeg_quality = 20;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }
  /* ---------------------------------------- */

  /* ---------------------------------------- Initialize camera */
  Serial.println();
  Serial.println("Camera initialization...");
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }
  Serial.print("Camera initialization was successful.");
  Serial.println();
  /* ---------------------------------------- */
  
 // capturePhotoSaveSpiffs(); //--> Calling the capturePhotoSaveSpiffs() subroutine.
  //sendPhoto("mobiswitch2021@gmail.com"); //--> Calling the sendPhoto() subroutine.
  
  Serial.println();
  Serial.println("Press the Reset button to :");
  Serial.println("- Reconnecting WiFi");
  Serial.println("- Take and send new photos.");
  Serial.println();

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // PLEASE PRESS THE "RESET" BUTTON ON THE ESP32-CAM BOARD TO RECONNECTING WIFI and TAKE AND SEND NEW PHOTOS. //
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
}
/* ________________________________________________________________________________ */

/* ________________________________________________________________________________ VOID LOOP */
void loop() {
  // put your main code here, to run repeatedly:
       if(WiFi.status() != WL_CONNECTED) { 
          digitalWrite(alert,LOW);
    //----------------------------------------
  }
  else{
      digitalWrite(alert,HIGH);
  
  }
  delay(1);
   int reading1 = digitalRead(buttonPin1);
    if ((reading1 != lastButton1State)){
      lastDebounceTime = millis();
    }
      if ((millis() - lastDebounceTime) > debounceDelay) {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:

    // if the button state has changed:
    if (reading1 != button1State) {
      button1State = reading1;

      // only toggle the LED if the new button state is HIGH
      if (button1State == HIGH) {
         capturePhotoSaveSpiffs(); //--> Calling the capturePhotoSaveSpiffs() subroutine.
  sendPhoto(emailRecipient);
      }
    }
}
  lastButton1State = reading1;

}
/* ________________________________________________________________________________ */
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
