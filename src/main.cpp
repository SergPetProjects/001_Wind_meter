// Программа для работы с анеморумбометром М63М-1

#include <Arduino.h>
#include <Wire.h>              //  Подключаем библиотеку для работы с шиной I2C
#include <LiquidCrystal_I2C.h> //  Подключаем библиотеку для работы с LCD дисплеем по шине I2C
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <Adafruit_BMP280.h>

#define _MEASUREMENT_PERIOD 10000  // Период измерения скорости ветра(10 с)
#define _MEASUREMENT_AZIMUTH 10000 // Период вычисления азимута флюгарки(2 мин = 120000)
#define _MEASUREMENT_TREND 22000   // Период вычисления тенденции погоды(10 мин = 600000)
#define _ARRAY_PERIOD_VOLUME 10    // Размер массива для хранения скорости ветра (60 = 10 мин / 10000 мс)
#define _ARRAY_FAZA_VOLUME 600     // Размер массива для хранения азимута (600)
#define _STEP 5                    // Разбиение по азимутам с шагом 5 градусов
#define _WIND_COEFFICIENT 1.3      // Коэффициент отношения скорости ветра к частоте вращения вертушки
#define _CALM 0.3                  // Скорость ветра (м/с) при штиле
#define _REED_SWITCH_1_PIN 2       // Цифровой вывод, подключенный к геркону вертушки
#define _REED_SWITCH_2_PIN 3       // Цифровой вывод, подключенный к геркону флюгарки
#define _DHT11_PIN 4               // Цифровой вывод, подключенный к датчику DHT 11
#define _DHT22_PIN 5               // Цифровой вывод, подключенный к датчику DHT 11
#define _DHT11_TYPE DHT11          // DHT 11
#define _DHT22_TYPE DHT22          // DHT 22 (AM2302)

LiquidCrystal_I2C lcd(0x27, 20, 4); //  Объявляем  объект библиотеки, указывая параметры дисплея (адрес I2C = 0x27, количество столбцов = 20, количество строк = 4)
Adafruit_BMP280 bmp;                // Датчик давления будет подключен по I2C
DHT_Unified dht_11(_DHT11_PIN, _DHT11_TYPE);
DHT_Unified dht_22(_DHT22_PIN, _DHT22_TYPE);

void isr();
void period();
void faza();
void bubbleSort();
void lcdFirst();
void lcdPrint();
void azimuth_array_copy();
void azimuthSort();
void azimuth();
void dataFormat();
void printSensorDetails(DHT_Unified dht);
float getTemperature(DHT_Unified dht);
float getHumidity(DHT_Unified dht);
void weatherTrend();
template <typename T>
String getTrend(T &val, T &val_old);

volatile bool period_flag = true; // разрешение на измерение периода
volatile bool faza_flag = false;  // разрешение на измерение фазы
volatile uint32_t tmr_period = 0, tmr_old = 0, tmr_faza, tmr_faza_old;
volatile uint32_t period_time, period_middle, period_min;
volatile uint16_t counter = 0;
uint32_t period_array[_ARRAY_PERIOD_VOLUME], sort_array[_ARRAY_PERIOD_VOLUME];
uint32_t azimuth_array[_ARRAY_FAZA_VOLUME], azimuth_faza[_ARRAY_FAZA_VOLUME];
uint32_t tmr2 = 0, tmr10s = 0, tmr10 = 0;
uint16_t azimuth_selection[360 / _STEP]; // 360 градусов разбито по 5 градусов
uint16_t i_period = 0, i_faza = 0, i_faza_max = 0, azimuth_middle = 0;
float wind, wind_max, wind_middle, wind_middle_old;
String azimuth_string;
// String azimuth_string_arr[] = {"CALM", "N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE", "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW", "N"};
String azimuth_string_arr[] = {"\5T\2\6\7", "  C", "CCB", " CB", "BCB", "  B", "B\2B", " \2B", "\2\2B", "  \2", "\2\2\6", " \2\6", "\6\2\6", "  \6", "\6C\6", " C\6", "CC\6", "  C"};
// String azimuth_ru = "DIRECTION";
// String azimuth_ru = "HA\3PAB\6.";     // НАПРАВЛ.

// const char bell[8] PROGMEM = {B00100, B01110, B01110, B01110, B11111, B00000, B00100, B00000};     // колокольчик
// const char litter_ZH[8] PROGMEM = {B10101, B10101, B10101, B01110, B10101, B10101, B10101, B00000}; // Ж
const char litter_o[8] PROGMEM = {B00111, B00101, B00111, B00000, B00000, B00000, B00000, B00000};    // o -0
const char litter_YU[8] PROGMEM = {B10010, B10101, B10101, B11101, B10101, B10101, B10010, B00000};   // Ю -2 (меняется динамически)
const char litter_I[8] PROGMEM = {B10001, B10001, B10011, B10101, B11001, B10001, B10001, B00000};    // И -2 (меняется динамически)
const char litter_down[8] PROGMEM = {B00100, B00100, B00100, B00100, B10101, B01110, B00100, B00000}; // стрелка вниз -3
const char litter_up[8] PROGMEM = {B00100, B01110, B10101, B00100, B00100, B00100, B00100, B00000}; // стрелка вверх -4
// const char litter_P[8] PROGMEM = {B11111, B10001, B10001, B10001, B10001, B10001, B10001, B00000}; // П -3
// const char litter_U[8] PROGMEM = {B10001, B10001, B10001, B01111, B00001, B00010, B01100, B00000};  // У
const char litter_SH[8] PROGMEM = {B10101, B10101, B10101, B10101, B10101, B10101, B11111, B00000}; // Ш -5
const char litter_L[8] PROGMEM = {B00011, B00101, B01001, B01001, B01001, B01001, B10001, B00000};  // Л -6 (меняется динамически)
const char litter_3[8] PROGMEM = {B01110, B10001, B00001, B00110, B00001, B10001, B01110, B00000};  // З -6 (меняется динамически)
const char litter_MZ[8] PROGMEM = {B10000, B10000, B10000, B11100, B10010, B10010, B11100, B00000}; // Ь -7

uint32_t delayDHT_11, delayDHT_22;
uint16_t pressure, pressure_old;
float temperature_in, temperature_out, humidity_in, humidity_out;
float temperature_in_old, temperature_out_old, humidity_in_old, humidity_out_old;
String temperature_in_trend, temperature_out_trend, humidity_in_trend, humidity_out_trend, pressure_trend, wind_middle_trend;
String trend_up = "\4", trend_down = "\3";

void setup()
{
  Serial.begin(9600);

  // Инициализация LCD
  lcdFirst();
  attachInterrupt(0, isr, FALLING);          // прерывание по LOW на D2
  attachInterrupt(1, faza, FALLING);         // прерывание по LOW на D3
  pinMode(_REED_SWITCH_1_PIN, INPUT_PULLUP); // назначить выводу порт ввода с подтяжкой
  pinMode(_REED_SWITCH_2_PIN, INPUT_PULLUP); // назначить выводу порт ввода с подтяжкой
  lcd.createChar(0, litter_o);
  // lcd.createChar(1, litter_3);
  lcd.createChar(2, litter_I);
  lcd.createChar(3, litter_down);
  lcd.createChar(4, litter_up);
  lcd.createChar(5, litter_SH);
  lcd.createChar(6, litter_L);
  lcd.createChar(7, litter_MZ);

  // Инициализация DHT
  dht_11.begin();
  dht_22.begin();
  // Вывод в консоль информации о датчиках DHTxx
  printSensorDetails(dht_11);
  printSensorDetails(dht_22);

  // Инициализация BMP 280
  bmp.begin(0x76);
  /* Настройки по умолчанию из технического описания. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Режим работы. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling - Передискретизация температуры*/
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling - Передискретизация давления */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
}

void loop()
{
  if (millis() - tmr10s >= _MEASUREMENT_PERIOD) // таймер 10 секунд
  {
    period(); // Расчёт периода вращения вертушки
    tmr10s = millis();

    period_array[i_period] = period_time; // заполнение массива данными о периоде (затирается самое старое)
    if (++i_period >= _ARRAY_PERIOD_VOLUME)
      i_period = 0;

    memcpy(sort_array, period_array, _ARRAY_PERIOD_VOLUME * sizeof(period_array[0])); // копирует массив period_array в sort_array
    bubbleSort();
    period_middle = sort_array[_ARRAY_PERIOD_VOLUME / 2];
    period_min = sort_array[2];

    // Измерение температуры и влажности
    temperature_in = getTemperature(dht_11);
    temperature_out = getTemperature(dht_22);
    humidity_in = getHumidity(dht_11);
    humidity_out = getHumidity(dht_22);

    dataFormat(); // преобразование данных перед выводом на дисплей
    lcdPrint();   // вывод на LCD
  }

  if (millis() - tmr2 >= _MEASUREMENT_AZIMUTH) // таймер 2 мин
  {
    azimuth_array_copy();
    azimuthSort(); // определение азимута по таймеру (2 мин)
    tmr2 = millis();
  }

  if (millis() - tmr10 >= _MEASUREMENT_TREND) // таймер 10 мин
  {
    weatherTrend(); // определение тенденции изменения погоды каждые 10 мин
    tmr10 = millis();
  }
}

// Счёт импульсов от D2 для вычисления периода вращения вертушки
void isr()
{
  if (period_flag)
  {
    tmr_period = micros();
    period_flag = false;
    faza_flag = true; // Включается ожидание импульса от D3 для вычисления фазы (азимута)
    counter++;        // импульсы считаем тут
    Serial.print("Количество импульсов за 10 сек. = ");
    Serial.println(counter);
  }
}

// Расчёт периода вращения вертушки
void period()
{
  uint32_t tmr1 = micros();
  if (counter != 0)
  {
    period_time = (tmr1 - tmr_old) / counter; // Расчёт периода вращения вертушки
    counter = 0;
    tmr_old = tmr1;
    Serial.print("Период вращения вертушки (мс) = ");
    Serial.println(period_time / 1000.0);
    Serial.println();
  }
  else
  {
    period_time = 4290000000; // вертушка не крутится (типа деление на 0)
  }
}

// Измерение фазы положения флюгарки (азимута)
// Работае по прерыванию на D3 (LOW) на основе предварительной фиксации времени импульса от D2
void faza()
{
  if (faza_flag)
  {
    tmr_faza_old = tmr_faza;
    tmr_faza = micros();
    azimuth(); // Расчёт азимута флюгарки
    period_flag = true;
    faza_flag = false;
  }
}

// Расчёт азимута флюгарки
void azimuth()
{
  azimuth_faza[i_faza] = (359 * (tmr_faza - tmr_period)) / (tmr_faza - tmr_faza_old); // азимут в градусах
  Serial.print("Азимут флюгарки = ");
  Serial.println(azimuth_faza[i_faza]);
  Serial.println();
  i_faza += (azimuth_faza[i_faza] <= 359) ? 1 : 0; // для затирания неверного значения азимута на следующей итерации
  if (i_faza >= _ARRAY_FAZA_VOLUME)
  {
    tmr2 = millis();
    azimuth_array_copy();
    azimuthSort(); // определение азимута при переполнении массива
  }
}

// Взятие актуальной части массива азимутов
void azimuth_array_copy()
{
  i_faza_max = i_faza;
  memcpy(azimuth_array, azimuth_faza, i_faza_max * sizeof(azimuth_faza[0])); // копирует актуальную часть массива azimuth_faza в azimuth_array
  i_faza = 0;
}

// Выборка из массива фаз самого частого значения азимута с дискретностью 5 градусов
void azimuthSort()
{
  uint16_t max = 0;
  for (uint16_t i = 0; i < 360 / _STEP; i++) // 360 / _STEP = 72 (для шага в 5 градусов)
    azimuth_selection[i] = 0;                // обнуление массива

  for (uint16_t i = 0; i < i_faza_max; i++)
  {
    uint32_t select = azimuth_array[i] / _STEP;
    if (max < ++azimuth_selection[select])
    {
      max = azimuth_selection[select]; // поиск max в массиве
      azimuth_middle = select * _STEP;
      Serial.print("azimuth_middle = ");
      Serial.println(azimuth_middle);
    }
    Serial.print(select * _STEP);
    Serial.print(" - ");
    Serial.println(azimuth_selection[select]);
  }
}

// Сортировка массива (пузырьковая)
void bubbleSort()
{
  for (int j = 0; j + 1 < _ARRAY_PERIOD_VOLUME; ++j)
  {
    for (int i = 0; i + 1 < _ARRAY_PERIOD_VOLUME - j; ++i)
    {
      if (sort_array[i] > sort_array[i + 1])
      {
        unsigned long temp = sort_array[i];
        sort_array[i] = sort_array[i + 1];
        sort_array[i + 1] = temp;
      }
    }
  }
}

// Подготовка данных к выводу на дисплей
void dataFormat()
{
  wind = 1000000.0 * _WIND_COEFFICIENT / period_time;          // мгновенное значение ветра
  wind_middle = 1000000.0 * _WIND_COEFFICIENT / period_middle; // среднее значение ветра за последние 2 минуты
  wind_max = 1000000.0 * _WIND_COEFFICIENT / period_min;       // максимальное значение ветра за последние 2 минуты (порыв)
  if (wind_middle < _CALM)                                     // Если штиль
  {
    lcd.createChar(2, litter_I); // переопределение "своих" символов LCD
    lcd.createChar(6, litter_L);
    delay(5);
    azimuth_string = azimuth_string_arr[0];
  }
  else
  {
    lcd.createChar(2, litter_YU); // переопределение "своих" символов LCD
    lcd.createChar(6, litter_3);
    delay(5);
    azimuth_string = azimuth_string_arr[1 + (int)((azimuth_middle + 11) / 22.5)];
  }
  pressure = bmp.readPressure() / 133.322;
}

// Первоначальный вывод на экран LCD
void lcdFirst()
{
  lcd.init(); // initialize the lcd
  lcd.backlight();
  lcd.setCursor(7, 0);
  lcd.print("METEO");

  lcd.setCursor(4, 3);
  lcd.print("Serg Grebnev");

  delay(1000);
  lcd.clear();
}

// Вывод информации на экран LCD
void lcdPrint()
{
  lcd.setCursor(0, 0);
  lcd.print(wind, 1);
  lcd.print("  ");

  lcd.setCursor(8, 0);
  lcd.print(wind_middle_trend);
  lcd.print(wind_middle, 1);
  lcd.print("  ");

  lcd.setCursor(16, 0);
  lcd.print(wind_max, 1);
  lcd.print("  ");

  lcd.setCursor(0, 1);
  lcd.print(azimuth_middle);
  lcd.write(223);
  lcd.print("   ");

  lcd.setCursor(6, 1);
  lcd.print("     ");
  lcd.setCursor(6, 1);
  lcd.print(azimuth_string);

  lcd.setCursor(12, 1);
  lcd.print(pressure_trend);
  lcd.print(pressure);
  lcd.print("mmHg");

  lcd.setCursor(0, 2);
  lcd.print(temperature_in_trend);
  lcd.print(temperature_in, 1);
  lcd.write(0);
  lcd.print("C  ");

  lcd.setCursor(11, 2);
  lcd.print(temperature_out_trend);
  lcd.print(temperature_out, 1);
  lcd.write(0);
  lcd.print("C  ");

  lcd.setCursor(0, 3);
  lcd.print(humidity_in_trend);
  lcd.print(humidity_in, 1);
  lcd.print("%  ");

  lcd.setCursor(11, 3);
  lcd.print(humidity_out_trend);
  lcd.print(humidity_out, 1);
  lcd.print("%  ");
}

// Print temperature sensor details.
void printSensorDetails(DHT_Unified dht)
{
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  Serial.println(F("------------------------------------"));
  Serial.println(F("Temperature Sensor"));
  Serial.print(F("Sensor Type: "));
  Serial.println(sensor.name);
  Serial.print(F("Driver Ver:  "));
  Serial.println(sensor.version);
  Serial.print(F("Unique ID:   "));
  Serial.println(sensor.sensor_id);
  Serial.print(F("Max Value:   "));
  Serial.print(sensor.max_value);
  Serial.println(F("°C"));
  Serial.print(F("Min Value:   "));
  Serial.print(sensor.min_value);
  Serial.println(F("°C"));
  Serial.print(F("Resolution:  "));
  Serial.print(sensor.resolution);
  Serial.println(F("°C"));
  Serial.println(F("------------------------------------"));
  // Print humidity sensor details.
  dht.humidity().getSensor(&sensor);
  Serial.println(F("Humidity Sensor"));
  Serial.print(F("Sensor Type: "));
  Serial.println(sensor.name);
  Serial.print(F("Driver Ver:  "));
  Serial.println(sensor.version);
  Serial.print(F("Unique ID:   "));
  Serial.println(sensor.sensor_id);
  Serial.print(F("Max Value:   "));
  Serial.print(sensor.max_value);
  Serial.println(F("%"));
  Serial.print(F("Min Value:   "));
  Serial.print(sensor.min_value);
  Serial.println(F("%"));
  Serial.print(F("Resolution:  "));
  Serial.print(sensor.resolution);
  Serial.println(F("%"));
}

// Получить значение температуры
float getTemperature(DHT_Unified dht)
{
  sensors_event_t event;
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature))
  {
    Serial.println(F("Error reading temperature!"));
  }
  else
  {
    Serial.print(F("Temperature: "));
    Serial.print(event.temperature);
    Serial.println(F("°C"));
  }
  return event.temperature;
}

// Получить значение влажности
float getHumidity(DHT_Unified dht)
{
  sensors_event_t event;
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity))
  {
    Serial.println(F("Error reading humidity!"));
  }
  else
  {
    Serial.print(F("Humidity: "));
    Serial.print(event.relative_humidity);
    Serial.println(F("%"));
  }
  return event.relative_humidity;
}

// Определение трендов параметров погоды
void weatherTrend()
{
  wind_middle_trend = getTrend(wind_middle, wind_middle_old);
  temperature_in_trend = getTrend(temperature_in, temperature_in_old);
  temperature_out_trend = getTrend(temperature_out, temperature_out_old);
  humidity_in_trend = getTrend(humidity_in, humidity_in_old);
  humidity_out_trend = getTrend(humidity_out, humidity_out_old);
  pressure_trend = getTrend(pressure, pressure_old);
}

template <typename T>
String getTrend(T &val, T &val_old)
{
  String result;
  if (val > val_old)
  {
    result = trend_up;
  }
  else if (val < val_old)
  {
    result = trend_down;
  }
  else
  {
    result = " ";
  }
  val_old = val;
  return result;
}