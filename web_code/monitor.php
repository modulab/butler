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
	  .you {background: #ffcccc}
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
	<div class="container-narrow">
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

	echo "<p> Your are <b class=\"text-info\">position $position</b> in the queue:</p>";
	if ($is_active==1) {
		echo "<p>I've picked your order, and am on my way :-)</p>";
	}
    echo "<p><table border=1>";
    echo "<tr><th>Status</th><th>Name</th><th>How Many Beers</th><th>Drink Station</th><th>Order Number</th></tr>";
    foreach ($orders as $order) {
        $is_active=0;


       $entry = explode(" ", $order,4);
    	foreach ($actives as $active) {
	    	if (intval($active) == intval($entry[0])) {
		      $is_active = 1;
		      break;
	    	}
	    }

       if ($entry[0] == $order_no) {
          echo "<tr class=\"you\"> ";
	   } else {
           echo "<tr>";
       }
       echo "<td>";
       if ($is_active==1){
           echo "<span class=\"badge badge-info\">On Route</span>";
       }
       echo "</td>";
       echo "<td>";
       echo $entry[3];
       echo "</td>";

       echo "<td>";
       echo $entry[2];
       echo "</td>";

       echo "<td>";
       echo $entry[1];
       echo "</td>";
       echo "<td>";
       echo $entry[0];
       echo "</td>";
       echo "</tr>";
    }
    echo "</table>";
	flock($lock, LOCK_UN);	
} else {
	echo "Error 42.";	
}
fclose($lock);

?>
<hr/>
<p class="masthead">
    
      <span class="text-error">ADAM</span> - the 
      <span class="muted">Autonomous Drinks Administering Machine</span> <small>from <a href="http://www.cs.bham.ac.uk/go/irlab">the Intelligent Robotics Lab</a> in the <a href="http://www.cs.bham.ac.uk/">School of Computer Science</a>, University of Birmingham.</small>
    
  </p>
<p >
<small>  <a href="about.php">More information</a>
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
