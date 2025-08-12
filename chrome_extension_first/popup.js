document.getElementById('sendEmail').addEventListener('click', function() {
    const note = document.getElementById('note').value;
    // Implement email sending logic here
    console.log('Sending email:', note);
  });
  
  document.getElementById('sendToNotion').addEventListener('click', function() {
    const note = document.getElementById('note').value;
    // Implement Notion API logic here
    console.log('Sending to Notion:', note);
  });