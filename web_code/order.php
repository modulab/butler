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
	<div class="container-narrow">		

<?php 
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

echo "<h1>Great, see you soon $name!</h1>";

echo "<p class=\"lead\"> Your order has been placed, and will be with you shortly. </p>";

echo "<p> Can't wait? Watch the queue <a href='monitor.php?id=$order_number'>here</a></p>";
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
