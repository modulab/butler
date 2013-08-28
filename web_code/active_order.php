<html>
<head>
<title>Order a Drink</title>
</head>
<body>
<h1>storing active orders</h1>
<?php 
$lock = fopen("data/lock","w+");
if (flock($lock, LOCK_EX)) {

	$active_orders=$_GET["active_orders"];

	/////////////////////////////
	// Write the active orders to file

	// Open orders file
	if (!$file = fopen("data/active.txt", 'w')) { 
	  print "Oh No! Err Code=1"; 
	  exit; 
	} 

	$orders=explode(".",$active_orders);
	foreach ($orders as $order) {
	  fwrite($file, $order."\n");
	}

	fclose($file);
	/////////////////////////////////////
	flock($lock, LOCK_UN);	
} else {
	echo "Error 42.";
}
fclose($lock);
echo "Ok. $active_orders";
?>

</body>
</html>
