negativeFolder ='C:\Users\marco\Documents\Proyectos\Haar\negative';
negativeImages = imageDatastore(negativeFolder);
trainCascadeObjectDetector('ballDetector.xml',positiveInstances, negativeFolder,'ObjectTrainingSize',[24,24],'FalseAlarmRate',0.4,'NumCascadeStages',18, 'FeatureType','LBP')