// Программа для работы с анеморумбометром М63М-1

#include <Arduino.h>
#include <Wire.h>              //  Подключаем библиотеку для работы с шиной I2C
#include <LiquidCrystal_I2C.h> //  Подключаем библиотеку для работы с LCD дисплеем по шине I2C

#define _MEASUREMENT_PERIOD 10000 // Период измерения (мс)
#define _ARRAY_VOLUME 60          // Размер массива для хранения скорости ветра (60 = 10 мин / 10000 мс)

LiquidCrystal_I2C lcd(0x27, 20, 4); //  Объявляем  объект библиотеки, указывая параметры дисплея (адрес I2C = 0x27, количество столбцов = 20, количество строк = 4)

void isr();
void period();
void faza();
void bubbleSort();
void lcdFirst();
void lcdPrint();

volatile bool period_flag = false; // разрешение на измерение периода
volatile bool faza_flag = false;   // разрешение на измерение фазы
volatile bool sort_flag = true;    // разрешение на сортировку массива
volatile uint32_t tmr_period = 0, tmr_old = 0, tmr_faza;
volatile uint32_t period_time, period_middle, period_max;
volatile uint16_t counter = 0, azimuth_faza = 0;
uint32_t period_array[_ARRAY_VOLUME], sort_array[_ARRAY_VOLUME];
uint32_t tmr = 0;
uint8_t i = 0;

void setup()
{
  Serial.begin(115200);
  lcdFirst();
  attachInterrupt(0, isr, FALLING);  // прерывание по LOW на D2
  attachInterrupt(1, faza, FALLING); // прерывание по LOW на D3
  pinMode(2, INPUT_PULLUP);          // назначить выводу порт ввода с подтяжкой
  pinMode(3, INPUT_PULLUP);          // назначить выводу порт ввода с подтяжкой
}

void loop()
{
  if (millis() - tmr >= _MEASUREMENT_PERIOD)
  {
    period(); // Расчёт периода вращения вертушки
    tmr = millis();
    period_flag = true; // разрешает запись времени прихода импульса от D2

    period_array[i] = period_time; // заполнение массива данными о периоде (затирается самое старое)
    if (++i >= _ARRAY_VOLUME)
      i = 0;

    if (sort_flag)
      bubbleSort();

    lcdPrint(); //вывод на LCD
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
    Serial.print("Время импульса D2 = ");
    Serial.println(tmr_period);
  }
  counter++; // импульсы считаем тут
  Serial.print("Количество импульсов = ");
  Serial.println(counter);
}

// Расчёт периода вращения вертушки
void period()
{
  uint32_t tmr1 = micros();
  if (counter == 0)
  {
    counter = 1;
  }
  period_time = (tmr1 - tmr_old) / counter; // Расчёт периода вращения вертушки
  counter = 0;
  tmr_old = tmr1;
  Serial.print("Период вращения вертушки = ");
  Serial.println(period_time / 1000000.0);
  Serial.println();
}

// Измерение фазы положения флюгарки (азимута)
// Работае по прерыванию на D3 (LOW) на основе предварительной фиксации времени импульса от D2
void faza()
{
  if (faza_flag)
  {
    tmr_faza = micros();
    azimuth_faza = 360 * (tmr_faza - tmr_period) / period_time; // азимут в градусах
    Serial.print("Время импульса D3 = ");
    Serial.println(tmr_faza);
    Serial.print("Азимут флюгарки = ");
    Serial.println(azimuth_faza);
    Serial.println();
    faza_flag = false;
  }
}

// Сортировка массива (пузырьковая)
void bubbleSort()
{
  sort_flag = false;
  memcpy(sort_array, period_array, _ARRAY_VOLUME * 4); // копирует массив period_array в sort_array
  for (int j = 0; j + 1 < _ARRAY_VOLUME; ++j)
  {
    for (int i = 0; i + 1 < _ARRAY_VOLUME - j; ++i)
    {
      if (sort_array[i] > sort_array[i + 1])
      {
        unsigned long temp = sort_array[i];
        sort_array[i] = sort_array[i + 1];
        sort_array[i + 1] = temp;
      }
    }
  }
  period_middle = sort_array[_ARRAY_VOLUME / 2];
  period_max = sort_array[_ARRAY_VOLUME - 3];
  sort_flag = true;
}

// Первоначальный вывод на экран LCD
void lcdFirst(){
  lcd.init(); // initialize the lcd
  lcd.backlight();
  lcd.setCursor(7, 0); //  Устанавливаем курсор в позицию (столбец, строка)
  lcd.print("METEO");
  lcd.setCursor(4, 3); //  Устанавливаем курсор в позицию (столбец, строка)
  lcd.print("Serg Grebnev");
  delay(1000);
  lcd.clear();
  lcd.setCursor(0, 0); //  Устанавливаем курсор в позицию (столбец, строка)
  lcd.print("BETEP:");
  lcd.setCursor(0, 1); //  Устанавливаем курсор в позицию (столбец, строка)
  lcd.print("  ==>");
  lcd.setCursor(0, 2); //  Устанавливаем курсор в позицию (столбец, строка)
  lcd.print("A3NMYT:");
}

// Вывод информации на экран LCD
void lcdPrint(){
  lcd.setCursor(8, 0); //  Устанавливаем курсор в позицию (столбец, строка)
  lcd.print(period_middle);
  lcd.setCursor(8, 1); //  Устанавливаем курсор в позицию (столбец, строка)
  lcd.print(period_max);
  lcd.setCursor(8, 2); //  Устанавливаем курсор в позицию (столбец, строка)
  lcd.print(azimuth_faza);
}