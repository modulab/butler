<html>
<head>
<title>Order a Drink</title>
</head>
<body>
<h1>I'm working on it!</h1>
<?PHP 
$lock = fopen("data/lock","w+");
if (flock($lock, LOCK_EX)) {
	$order_no=$_GET["id"];

	//////////////////////////////
	// Read the orders
	$orders = file("data/orders.txt");
	$position=0;
	$set=0;
	foreach ($orders as $order) {
		$position = $position +1;
		$entry = explode(" ", $order,2);
		if ($entry[0] == $order_no) {
		  $set=1;
		  break;
		}
	}
	if ($set==0) {
		echo "ERROR - Order number does not exist!";
		exit;
	}
	//////////////////////////////

	//////////////////////////////
	// approximate time per order
	$times=file("data/closures.txt");
	$number=0;
	$total=0;
	foreach ($times as $time) {
		$total = intval($time) - intval($times[0]);
		$number = $number + 1;
	}
	$average = $total / $number;
	//////////////////////////////

	/////////////////////////////
	// find out what orders are active
	$actives = file("data/active.txt");
	$is_active=0;
	foreach ($actives as $active) {
		if (intval($active) == intval($order_no)) {
		  $is_active = 1;
		  break;
		}
	}
	/////////////////////////////

	$eta = $position * $average;

	echo "<p> Your are order number $order_no, estimated time to delivery <b>$eta seconds</b>. </p>";
	if ($is_active==1) {
		echo "<p>I've picked your order, and am on my way :-)</p>";
	}
	flock($lock, LOCK_UN);	
} else {
	echo "Error 42.";	
}
fclose($lock);

?>

</body>
</html>
