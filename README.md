# Regulator temperatury PID
## TODO list
> Implementacja regulatora PID

> Wyświetlanie aktualnej temp. oraz zadanej na wyświetlaczu LCD
## Opis projektu
Mikroprocesorowy system sterowania i pomiaru w oparciu o mikrokontroler z rodziny STM32 oraz elementy wykonawcze i pomiarowe. Program realizuje możliwość sterowania temperaturą rezystora 5W za pomocą regulatora PID. Dodatkowo wykorzystywany jest wyświetlacz LCD do pokazywania temperatury zadanej oraz aktualnej, którą jesteśmy w stanie odczytać przy pomocy czujnika BMP280 wykorzystując komunikację I2C.
## Sprzęt i oprogramowanie
- moduł Nucleo STM32F746
- czujnik BMP280
- niskoomowy rezystor ogrzewający 5W (39 Ω)
- tranzystor
- rezystory 100kΩ oraz 1kΩ
- środowisko STM32CubeIDE
