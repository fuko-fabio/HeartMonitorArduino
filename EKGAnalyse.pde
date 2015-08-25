
const int ledPin =  13;      // the number of the LED pin
const int led2Pin = 2;
const int ecgSignal = 5;

unsigned long previousTime = 0;
unsigned long currentTime = 0;
unsigned long deltaTime = 0;
unsigned long frameCounter = 0; // main loop executive frame counter

int lastSignal = 0;
int groundSignal = 0;
int highPassSignal = 0;
int lowPassSignal = 0;
int derivativeSignal = 0;
int powSignal = 0;
int movingWindowSignal = 0;
double scaledSignal = 0;

bool loopLock = true;

int AREF = 3.3;
double ecgScaleFactor = AREF/1024.0;

char readChar();

void setup()
{
	pinMode(ledPin, OUTPUT);  
	digitalWrite(ledPin, LOW);
	Serial.begin(115200);
	analogReference(EXTERNAL);

	while (true)
	{
		if(readChar() == 's')
		{
			loopLock = false;
			break;
		}
	}
}

void loop()
{
	if(!loopLock)
		{
		currentTime = micros();
		deltaTime = currentTime - previousTime;

		// Main scheduler loop set for 200hz (5000)
		if (deltaTime >= 5000)
		{
			frameCounter++;
			if (frameCounter %   1 == 0)
			{
				lastSignal = analogRead(ecgSignal);
				lowPassSignal = lowPassFilter(lastSignal);
				highPassSignal = highPassFilter(lowPassSignal);
				scaledSignal = highPassSignal* ecgScaleFactor;

				Serial.println(scaledSignal,5);

				derivativeSignal = derivative(highPassSignal);
				powSignal = derivativeSignal * derivativeSignal;
				movingWindowSignal = movingWindowIntegral(powSignal);

				if(movingWindowSignal > 100)
					digitalWrite(led2Pin, HIGH);
				else
					digitalWrite(led2Pin, LOW);
				//Serial.println(groundSignal* ecgScaleFactor);
			}
			if (frameCounter %   2 == 0) //  100 Hz tasks
			{
				//Serial.println(scaledSignal,5);
			}

			if (frameCounter %   4 == 0) //  50 Hz tasks
			{
				//Serial.println(scaledSignal,5);
			}

			if (frameCounter %   10 == 0) //  20 Hz tasks
			{
				//Serial.println(modSignal);			
			}
			previousTime = currentTime;
		}
		if (frameCounter >= 100) 
		frameCounter = 0;
	}
	if(readChar() == 'p')
	{
		loopLock = true;
	}
	if(readChar() == 's')
	{
		loopLock = false;
	}
}

int lowPassFilter(int data)
{
	static int y1 = 0, y2 = 0, x[26], n = 12;
	int y0;
	x[n] = x[n + 13] = data;
	y0 = (y1 << 1) - y2 + x[n] - (x[n + 6] << 1) + x[n + 12];
	y2 = y1;
	y1 = y0;
	y0 >>= 5;
	if(--n < 0)
	n = 12;
	return(y0);
}

int highPassFilter(int data)
{
	static int y1 = 0, x[66], n = 32;
	int y0;
	x[n] = x[n + 33] = data;
	y0 = y1 + x[n] - x[n + 32];
	y1 = y0;
	if(--n < 0)
	n = 32;
	return(x[n + 16] - (y0 >> 5));
}

int derivative(int data)
{
	int y, i;
	static int x_derv[4];
	/*y = 1/8 (2x( nT) + x( nT - T) - x( nT - 3T) - 2x( nT -4T))*/
	y = (data << 1) + x_derv[3] - x_derv[1] - ( x_derv[0] <<1);
	y >>= 3;
	for (i = 0; i < 3; i++)
		x_derv[i] = x_derv[i + 1];
	x_derv[3] = data;
	return(y);
}

int movingWindowIntegral(int data)
{
	static int x[32], ptr = 0;
	static long sum = 0;
	long ly;
	int y;
	if(++ptr == 32)
		ptr = 0;
	sum -= x[ptr];
	sum += data;
	x[ptr] = data;
	ly = sum >> 5;
	if(ly > 32400) /*check for register overflow*/
		y = 32400;
	else
		y = (int) ly;
	return(y);
}

char readChar()
{
	int incomingByte;	// for incoming serial data

	if (Serial.available() > 0)
	{
		// read the incoming byte:
		incomingByte = Serial.read();

		return incomingByte;
	}
	else
	{
		return 'a';
	}

}