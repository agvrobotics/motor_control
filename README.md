# Motor and Encoder Pin Configuration


| Wheel           | Function       | Arduino Pin | L298N Connection | Notes       |
|-----------------|----------------|------------|-----------------|------------|
| **Front Left (FL)**  | PWM (speed)   | 5          | ENA_F           |            |
|                 | Forward        | 22         | IN1_F           |            |
|                 | Reverse        | 23         | IN2_F           |            |
|                 | Encoder A      | 18         | —               | Yellow wire |
|                 | Encoder B      | 19         | —               | Green wire  |
| **Front Right (FR)** | PWM (speed)   | 6          | ENB_F           |            |
|                 | Forward        | 24         | IN3_F           |            |
|                 | Reverse        | 25         | IN4_F           |            |
|                 | Encoder A      | 20         | —               | Green wire |
|                 | Encoder B      | 21         | —               | Yellow wire |
| **Rear Left (RL)**   | PWM (speed)   | 7          | ENA_R           |            |
|                 | Forward        | 26         | IN1_R           |            |
|                 | Reverse        | 27         | IN2_R           |            |
|                 | Encoder A      | —          | —               | Not connected |
|                 | Encoder B      | —          | —               | Not connected |
| **Rear Right (RR)**  | PWM (speed)   | 8          | ENB_R           |            |
|                 | Forward        | 28         | IN3_R           |            |
|                 | Reverse        | 29         | IN4_R           |            |
|                 | Encoder A      | 2          | —               | Green wire |
|                 | Encoder B      | 3          | —               | Yellow wire |


