#include <WiFi.h>
#include <esp_wifi.h>
#include <Arduino.h>
#include "ESPNowW.h"
//#define ESPNow_debug
uint8_t Clu_mac[] = {0x36, 0x33, 0x33, 0x33, 0x33, 0x32};
uint8_t Gas_mac[] = {0x36, 0x33, 0x33, 0x33, 0x33, 0x33};
uint8_t Brk_mac[] = {0x36, 0x33, 0x33, 0x33, 0x33, 0x34};
uint8_t Host_mac[] = {0x36, 0x33, 0x33, 0x33, 0x33, 0x35};
uint8_t* Recv_mac;
uint16_t ESPNow_send=0;
uint16_t ESPNow_recieve=0;
//bool MAC_get=false;
bool ESPNOW_status =false;
bool ESPNow_initial_status=false;
bool ESPNow_update= false;
//https://github.com/nickgammon/I2C_Anything/tree/master
struct ESPNow_Send_Struct
{ 
  uint16_t pedal_position;
  float pedal_position_ratio;
};
ESPNow_Send_Struct _ESPNow_Recv;
ESPNow_Send_Struct _ESPNow_Send;

void onRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) 
{
  /*
  if(ESPNOW_status)
  {
    memcpy(&ESPNow_recieve, data, sizeof(ESPNow_recieve));
    ESPNow_update=true;
  }
  */
  if(ESPNOW_status)
  {
    memcpy(&_ESPNow_Recv, data, sizeof(_ESPNow_Recv));
    ESPNow_update=true;
  }

}
void OnSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{

}
void ESPNow_initialize()
{

    WiFi.mode(WIFI_MODE_STA);
    Serial.println("Initializing Rudder, please wait"); 
    Serial.print("Current MAC Address:  ");  
    Serial.println(WiFi.macAddress());
    if(dap_config_st.payLoadPedalConfig_.pedal_type==0)
    {
      esp_wifi_set_mac(WIFI_IF_STA, &Clu_mac[0]);
    }
    if(dap_config_st.payLoadPedalConfig_.pedal_type==1)
    {
      esp_wifi_set_mac(WIFI_IF_STA, &Brk_mac[0]);
    }
    if(dap_config_st.payLoadPedalConfig_.pedal_type==2)
    {
      esp_wifi_set_mac(WIFI_IF_STA, &Gas_mac[0]);
    }
    delay(300);
    Serial.print("Modified MAC Address:  ");  
    Serial.println(WiFi.macAddress());
    ESPNow.init();
    Serial.println("wait 10s for ESPNOW initialized");
    delay(10000);

    if(dap_config_st.payLoadPedalConfig_.pedal_type==1)
    {
      Recv_mac=Gas_mac;      
    }

    if(dap_config_st.payLoadPedalConfig_.pedal_type==2)
    {
      Recv_mac=Brk_mac;
    }
    if(ESPNow.add_peer(Recv_mac)== ESP_OK)
    {
      ESPNOW_status=true;
      Serial.println("Sucess to add peer");
    }
    else
    {
      ESPNOW_status=false;
      Serial.println("Fail to add peer");
    }
    ESPNow.reg_recv_cb(onRecv);
    ESPNow.reg_send_cb(OnSent);
    ESPNow_initial_status=true;
    Serial.println("Rudder Initialized");
  
}
