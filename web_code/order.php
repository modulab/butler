<?php 
// Check that the user got here via drink.php - form data required
$required = array('station_id', 'drinks', 'name');

// Loop over field names, make sure each one exists and is not empty
$error = false;
foreach($required as $field) {
  if (empty($_POST[$field])) {
    $error = true;
  }
}
if ($error) {
   echo "Error";
   exit;
}

$lock = fopen("data/lock","w+");
if (flock($lock, LOCK_EX)) {
	$station=$_POST["station_id"];
	$drinks=$_POST["drinks"];
	$name=$_POST["name"];

	$order_number=intval(file_get_contents("data/last.txt"))+1;

	file_put_contents("data/last.txt",$order_number);

	// Open orders file
	if (!$file = fopen("data/orders.txt", 'a')) { 
		print "Oh No! Err Code=1"; 
		exit; 
	} 

	if (!fwrite($file, "".$order_number." ".$station." ".$drinks." ".$name."\n" )) { 
		print "Oh No! Ordering problem...";
		exit; 
	} 
	fclose($file);
	flock($lock, LOCK_UN);	
} else {
	echo "Error 42.";	
}
$encoded_name=urlencode($name);
header("Location: http://cs.bham.ac.uk/~burbrcjc/bsf2013/placed_order.php?name=$encoded_name&order=$order_number");
exit;
?>
