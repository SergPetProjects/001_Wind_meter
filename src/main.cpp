// Программа для работы с анеморумбометром М63М-1

#include <Arduino.h>
#include <Wire.h>              //  Подключаем библиотеку для работы с шиной I2C
#include <LiquidCrystal_I2C.h> //  Подключаем библиотеку для работы с LCD дисплеем по шине I2C

#define _MEASUREMENT_PERIOD 10000   // Период измерения скорости ветра(10 с)
#define _MEASUREMENT_AZIMUTH 120000 // Период вычисления азимута флюгарки(2 мин)
#define _ARRAY_PERIOD_VOLUME 60     // Размер массива для хранения скорости ветра (60 = 10 мин / 10000 мс)
#define _ARRAY_FAZA_VOLUME 600      // Размер массива для хранения азимута (600)
#define _STEP 5                     // Разбиение по азимутам с шагом 5 градусов
#define _WIND_COEFFICIENT 1.3       // Коэффициент отношения скорости ветра к частоте вращения вертушки

LiquidCrystal_I2C lcd(0x27, 20, 4); //  Объявляем  объект библиотеки, указывая параметры дисплея (адрес I2C = 0x27, количество столбцов = 20, количество строк = 4)

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

volatile bool period_flag = true; // разрешение на измерение периода
volatile bool faza_flag = false;  // разрешение на измерение фазы
volatile uint32_t tmr_period = 0, tmr_old = 0, tmr_faza, tmr_faza_old;
volatile uint32_t period_time, period_middle, period_min;
volatile uint16_t counter = 0;
uint32_t period_array[_ARRAY_PERIOD_VOLUME], sort_array[_ARRAY_PERIOD_VOLUME];
uint32_t azimuth_array[_ARRAY_FAZA_VOLUME], azimuth_faza[_ARRAY_FAZA_VOLUME];
uint32_t tmr2 = 0, tmr10 = 0;
uint16_t azimuth_selection[360 / _STEP]; // 360 градусов разбито по 5 градусов
uint16_t i_period = 0, i_faza = 0, i_faza_max = 0, azimuth_middle = 0;
float wind, wind_max, wind_middle;

void setup()
{
  Serial.begin(9600);
  lcdFirst();
  attachInterrupt(0, isr, FALLING);  // прерывание по LOW на D2
  attachInterrupt(1, faza, FALLING); // прерывание по LOW на D3
  pinMode(2, INPUT_PULLUP);          // назначить выводу порт ввода с подтяжкой
  pinMode(3, INPUT_PULLUP);          // назначить выводу порт ввода с подтяжкой
}

void loop()
{
  if (millis() - tmr10 >= _MEASUREMENT_PERIOD)
  {
    period(); // Расчёт периода вращения вертушки
    tmr10 = millis();

    period_array[i_period] = period_time; // заполнение массива данными о периоде (затирается самое старое)
    if (++i_period >= _ARRAY_PERIOD_VOLUME)
      i_period = 0;

    memcpy(sort_array, period_array, _ARRAY_PERIOD_VOLUME * sizeof(period_array[0])); // копирует массив period_array в sort_array
    bubbleSort();
    period_middle = sort_array[_ARRAY_PERIOD_VOLUME / 2];
    period_min = sort_array[2];

    dataFormat(); // преобразование данных перед выводом на дисплей
    lcdPrint();   // вывод на LCD
  }

  if (millis() - tmr2 >= _MEASUREMENT_AZIMUTH)
  {
    azimuth_array_copy();
    azimuthSort(); // определение азимута по таймеру (2 мин)
    tmr2 = millis();
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
    period_time = 4290000000;
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

// преобразование данных перед выводом на дисплей
void dataFormat()
{
  wind = 1000000.0 * _WIND_COEFFICIENT / period_time;
  wind_middle = 1000000.0 * _WIND_COEFFICIENT / period_middle;
  wind_max = 1000000.0 * _WIND_COEFFICIENT / period_min;
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

  lcd.setCursor(0, 0);
  lcd.print("BETEP:");

  lcd.setCursor(0, 1);
  lcd.print("CPEDH.:");

  lcd.setCursor(0, 2);
  lcd.print("  ==>");

  lcd.setCursor(0, 3);
  lcd.print("A3NMYT:");
}

// Вывод информации на экран LCD
void lcdPrint()
{
  lcd.setCursor(8, 0);
  lcd.print(wind);
  lcd.print(" m/s   ");

  lcd.setCursor(8, 1);
  lcd.print(wind_middle);
  lcd.print(" m/s   ");

  lcd.setCursor(8, 2);
  lcd.print(wind_max);
  lcd.print(" m/s   ");

  lcd.setCursor(8, 3);
  lcd.print(azimuth_middle);
  lcd.print("   ");
}