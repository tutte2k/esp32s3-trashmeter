# esp32s3-trashmeter

- accelerometer task

  - skickar x data på queue

  - skickar event baserat på y data

- led strip task

  - lyssnar på event och shiftar in färger baserat på bits

- steppo master task

  - konsumerar queue

- luta accelerometer i x led för att öka antal steg som steppometern tar

  - skippar om otillräcklig lutning

- luta i y led så overridas vilket håll den steppar

  - shiftar in rött/grönt i strip beroende på vilket håll

- vid otillräcklig lutning i y led

  - shiftar in blått från andra hållet i strip

- vid blått i strip

  - steppomaester stannar

- om hela strippen blir blå

  - light sleep

- vid omstart
  - läses föregående strip state från nvs flash
