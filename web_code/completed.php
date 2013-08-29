<html>
<head>
<title>Order a Drink</title>
</head>
<body>
<h1>Internal; order completion/cancellation</h1>
<?PHP 
$lock = fopen("data/lock","w+");
if (flock($lock, LOCK_EX)) {
	$order_no=$_GET["order_number"];
	echo "Completing order number $order_no";

	/////////////////////////////////////
	// open the orders file and remove the line 

	// read the order lines
	$lines = file('data/orders.txt');

	// write back to the file
	if (!$file = fopen("data/orders.txt", 'w')) { 
		print "Oh No! Err Code=1"; 
		exit; 
	} 

	foreach ($lines as $line) {
	  $entry = explode(" ", $line,2);
	  if ($entry[0] != $order_no) {
	   fwrite($file, $line);
	  }
	}
	
	fclose($file);
	////////////////////////////////////

	////////////////////////////////////
	// append the timestamp to the closures file
	// write back to the file
	if (!$file = fopen("data/closures.txt", 'a')) { 
		print "Oh No! Err Code=1"; 
		exit; 
	}
	fwrite($file, time()."\n" );	
	fclose($file);
	///////////////////////////////////

	flock($lock, LOCK_UN);	
} else {
	echo "Error 42.";
}
fclose($lock);

?>

</body>
</html>
