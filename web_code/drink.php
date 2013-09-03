<!DOCTYPE html>
<html lang="en">
  <head>
    <meta charset="utf-8">
    <title>ADAM - Autonomous Drinks Administering Machine</title>
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <meta name="description" content="Autonomous Drinks Administering Machine">
    <meta name="author" content="School of Computer Science, University of Birmingham">

    <!-- Le styles -->
    <link href="./assets/css/bootstrap.css" rel="stylesheet">
    <style type="text/css">
      body {
        padding-top: 20px;
        padding-bottom: 40px;
      }

      /* Custom container */
      .container-narrow {
        margin: 0 auto;
        max-width: 700px;
      }
      .container-narrow > hr {
        margin: 30px 0;
      }

      /* Main marketing message and sign up button */
      .jumbotron {
        margin: 60px 0;
        text-align: center;
      }
      .jumbotron h1 {
        font-size: 72px;
        line-height: 1;
      }
      .jumbotron .btn {
        font-size: 21px;
        padding: 14px 24px;
      }

      /* Supporting marketing content */
      .marketing {
        margin: 60px 0;
      }
      .marketing p + h4 {
        margin-top: 28px;
      }
    </style>
    <link href="./assets/css/bootstrap-responsive.css" rel="stylesheet">

    <!-- HTML5 shim, for IE6-8 support of HTML5 elements -->
    <!--[if lt IE 9]>
      <script src="./assets/js/html5shiv.js"></script>
    <![endif]-->

    <!-- Fav and touch icons -->
    <link rel="apple-touch-icon-precomposed" sizes="144x144" href="./assets/ico/apple-touch-icon-144-precomposed.png">
    <link rel="apple-touch-icon-precomposed" sizes="114x114" href="./assets/ico/apple-touch-icon-114-precomposed.png">
      <link rel="apple-touch-icon-precomposed" sizes="72x72" href="./assets/ico/apple-touch-icon-72-precomposed.png">
                    <link rel="apple-touch-icon-precomposed" href="./assets/ico/apple-touch-icon-57-precomposed.png">
                                   <link rel="shortcut icon" href="./assets/ico/favicon.png">
  </head>
<body>
<script>
function validateForm()
{
var x=document.forms["orderform"]["name"].value;
if (x==null || x=="")
  {
  alert("A name must be given");
  return false;
  }
if (/[~`!#$_%\^&*+=\-\[\]\\';,/{}0123456789|\\":<>\?]/g.test(x)) {
alert("Please give a name without special characters.");
  return false;
 
}
}
</script>
<?PHP  
$station=$_GET["station"];

if ( ($station != '1') &&
     ($station != '2') &&
     ($station != '3') &&
     ($station != '4') ) {
echo "Error: bad station";
exit;
}
?>
<div class="container-narrow">
	<div class="masthead">
		<h3 class="muted"><?php 
			echo "Welcome to <span class=\"text-error\">ADAM</span> Station $station";
		?></h3>
	</div>

<?PHP
// Check the number of orders already
$lock = fopen("data/lock","w+");
if (flock($lock, LOCK_EX)) {

	//////////////////////////////
	// Read the orders
	$orders = file("data/orders.txt");
	$beer_count=0;
	foreach ($orders as $order) {
		$entry = explode(" ", $order);
		if ($entry[2] == "1beer") {
		  $beer_count=$beer_count+1;
		}
		if ($entry[2] == "2beer") {
		  $beer_count=$beer_count+2;
		}
		if ($entry[2] == "3beer") {
		  $beer_count=$beer_count+3;
		}
	}
	if ($beer_count>15) { 
//        // assume average 5 beers per trip, don't want to queue too many, ~5min per trip
		echo "<div class='alert'><strong>Warning!</strong> The current wait for a drink is approximately ";
        echo $beer_count;
        echo " minutes, so you may want to do something else and order later</div>";
	}

	flock($lock, LOCK_UN);	
} else {
	echo "Error 42.";	
}
fclose($lock);

?>

<form action="order.php" method="post" name="orderform" onsubmit="return validateForm()" >
	<fieldset>
		<legend>Please place your order:</legend>
<?php
echo "<input type='hidden' name='station_id' value=$station>";
?>
<label>Your name: (optional)</label>
<input type="text" name="name">
<label>How many beers? I can't carry more than 3...</label>

<select name="drinks">
<option value="1beer"selected>1</option>
<option value="2beer">2</option>
<option value="3beer" >3</option>
</select><br>

<button type="submit" class="btn btn-primary">Order</button>
</fieldset>
</form>
<hr/>
<p >
  <a href="about.php">More information</a>
</p>
</div>
 <!-- Le javascript
    ================================================== -->
    <!-- Placed at the end of the document so the pages load faster -->
    <script src="./assets/js/jquery.js"></script>
    <script src="./assets/js/bootstrap-transition.js"></script>
    <script src="./assets/js/bootstrap-alert.js"></script>
    <script src="./assets/js/bootstrap-modal.js"></script>
    <script src="./assets/js/bootstrap-dropdown.js"></script>
    <script src="./assets/js/bootstrap-scrollspy.js"></script>
    <script src="./assets/js/bootstrap-tab.js"></script>
    <script src="./assets/js/bootstrap-tooltip.js"></script>
    <script src="./assets/js/bootstrap-popover.js"></script>
    <script src="./assets/js/bootstrap-button.js"></script>
    <script src="./assets/js/bootstrap-collapse.js"></script>
    <script src="./assets/js/bootstrap-carousel.js"></script>
    <script src="./assets/js/bootstrap-typeahead.js"></script>

</body>
</html>
