void setup() {
  // put your setup code here, to run once:

Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:
    
  
  
  int dustDensity=45;
  int temp=54;
  int hum=75;
  int longitude=123.07;
  int lat=2333.32;
    
    
    char dustDensity_str[5];
    dtostrf(dustDensity,4,1,dustDensity_str);  //4 is mininum width, 3 is precision; float value is copied onto buff

    char temp_str[4];
    dtostrf(temp,3,1,temp_str);  //4 is mininum width, 3 is precision; float value is copied onto buff

    char hum_str[3];
    dtostrf(hum,3,1,hum_str);  //4 is mininum width, 3 is precision; float value is copied onto buff

    char long_str[3];
    dtostrf(longitude,3,1,long_str);  //4 is mininum width, 3 is precision; float value is copied onto buff


    char lat_str[3];
    dtostrf(lat,3,1,lat_str);  //4 is mininum width, 3 is precision; float value is copied onto buff
    
    int ldata=strlen(dustDensity_str)+strlen(hum_str)+strlen(temp_str)+strlen(long_str)+strlen(lat_str); // Length of string of all combined data
    
    char data_str[60];

    sprintf(data_str,"dust=%s&temp=%s&hum=%s&long=%s&lat=%s",dustDensity_str,temp_str,hum_str,long_str,lat_str);

    printf("Now sending %s...",data_str);
    Serial.println(data_str);
}


