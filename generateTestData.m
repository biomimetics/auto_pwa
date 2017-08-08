%%Function to generate sample data for data importing/processing
function generateTestData

	% IMU data
	t = 600:1000:1e7;
	tR = t' + 100*randn(length(t),1);
	dR = 5*sin(2*pi/62500.*tR);

	% Optitrak Data
	t = 600:10000:1e7;
	tO = t' + 1000*randn(length(t),1);
	dO = 5*sin(2*pi/62500.*tO);

	csvwrite('../test_data/testRobot.txt', [tR,dR]);
	csvwrite('../test_data/testOptitrak.txt', [tO,dO]);
