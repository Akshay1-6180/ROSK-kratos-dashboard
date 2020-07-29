sidedown()


function sidedown(){
  $('#sidebar').toggleClass('active');
        if($('#life').text()== "")
        {
            $('#life').html('<i class="fab fa-pagelines move"></i>LIFE DETECTION');
            $('#auto').html("<i class='fas fa-dharmachakra move'></i>AUTONOMOUS");
            $('#comm').html("<i class='fas fa-signal move'></i>COMMUNICATION");

            
        }
        else if ($('#life').text() == "LIFE DETECTION")
        {
            $('#life').html("<i class='fab fa-pagelines move'></i>");
            $('#auto').html("<i class='fas fa-dharmachakra move'></i>");
            $('#comm').html("<i class='fas fa-signal move'></i>");
        }

}

$(document).ready(function () {
    $('#sidebarCollapse').on('click', sidedown);
})

//AJAX CODE

$("#forward").click(function(){
  $.ajax({
      type: "GET",
      url: "http://127.0.0.1:5000/forward",
      crossDomain: true,
      headers : {'Access-Control-Allow-Origin' : "http://127.0.0.1:5000/forward"}
  })
  .done(function(data){
      console.log(data)
  })
  .fail(function(data){
      console.log("fail")
  })
})

$("#backward").click(function(){
  $.ajax({
      type: "GET",
      url: "http://127.0.0.1:5000/backward",
      dataType: 'json'
  })
  .done(function(data){
      console.log(data)
  })
  .fail(function(data){
      console.log("fail")
  })
})

$("#left").click(function(){
  $.ajax({
      type: "GET",
      url: "http://127.0.0.1:5000/left",
      dataType: 'json'
  })
  .done(function(data){
      console.log(data)
  })
  .fail(function(data){
      console.log("fail")
  })
})

$("#right").click(function(){
  $.ajax({
      type: "GET",
      url: "http://127.0.0.1:5000/right",
      dataType: 'json'
  })
  .done(function(data){
      console.log(data)
  })
  .fail(function(data){
      console.log("fail")
  })
})

$("#stop").click(function(){
  $.ajax({
      type: "GET",
      url: "http://127.0.0.1:5000/stop",
      dataType: 'json'
  })
  .done(function(data){
      console.log(data)
  })
  .fail(function(data){
      console.log("fail")
  })
})


$("#video1").click(function(){
  $.ajax({
      type: "POST",
      url: "http://127.0.0.1:5000/video",
      headers : {'Access-Control-Allow-Origin' : "http://127.0.0.1:5000/video"}
  })
  .done(function(data){
      console.log(data)
      $('#videostream1').attr('src',"http://127.0.0.1:5000/video_feed")
  })
  .fail(function(data){
      console.log("fail")
  })
})
$("#video2").click(function(){
    $.ajax({
        type: "POST",
        url: "http://127.0.0.1:5000/video",
        headers : {'Access-Control-Allow-Origin' : "http://127.0.0.1:5000/video"}
    })
    .done(function(data){
        console.log(data)
        $('#videostream2').attr('src',"http://127.0.0.1:5000/video_feed")
    })
    .fail(function(data){
        console.log("fail")
    })
  })

  $("#graph").click(function(){
    var ic = 0;
    function getData(){
      var d1 = [];
      for (var i = 0 ; i < 19 ; i += 0.1)
        d1.push([i, Math.sin(i - ic)]);
      if (ic==12.5) {ic=0;} else {ic = ic + 0.05;}
      ic = Math.round(ic*100)/100;
      return d1;
    }
    function update(){
        plot.setData([getData()]);
        plot.draw();
      }      
    
    $("#graphs").html("<div id='myChart' style='width:640px;height:300px;'></div> <div class='card-body d-flex justify-content-center'><button id='graph' type='button' class='py-3 d-flex align-items-center btn btn-success'>LIVE GRAPH</button></div>")
        
        
        var plot = $.plot($("#myChart"), [ getData()]);
        var int = self.setInterval(function(){update()},50);
  })


//GRAPH POLTING CODE USING D3 AND JAVASCRIPT




