class PID {
public:

  PID(double, double, double, double);

  void setKs(double, double, double);

  void setKp(double);
  void setKi(double);
  void setKd(double);

  double run(double);

private:
  unsigned long before;
  double goal;
  double errorSum, lastError;
  double Kp, Ki, Kd;
};