<html>
<head>
<title>Order a Drink</title>
</head>
<body>
<?PHP 
$station=$_GET["station"];
?>

<h1><?php 
echo "Welcome to Station $station";
?></h1>
<p> Please place your order: </p>
<form action="order.php" method="post">
<?php
echo "<input type='hidden' name='station_id' value=$station>";
?>
Your name: <input type="text" name="name"><br>
How many beers? I can't carry more than 3..
<select name="drinks">
<option value="1beer"selected>1</option>
<option value="2beer">2</option>
<option value="3beer" >3</option>
</select><br>

<input type="submit">
</form>
</body>
</html>
