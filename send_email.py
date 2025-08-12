import smtplib
from email.mime.text import MIMEText
from email.mime.multipart import MIMEMultipart


def send_email():
    sender_email = ""
    receiver_email = ""
    password = raw_input("Enter your email password: ")  # It's safer to input the password at runtime

    message = MIMEMultipart()
    message["From"] = sender_email
    message["To"] = receiver_email
    message["Subject"] = "Test Email"

    body = "This is a test email sent from Python."
    message.attach(MIMEText(body, "plain"))

    try:
        server = smtplib.SMTP("smtp.gmail.com", 587)
        server.starttls()
        server.login(sender_email, password)
        server.sendmail(sender_email, receiver_email, message.as_string())
        server.quit()
        print "Email sent successfully!"
    except Exception as e:
        print "An error occurred: %s" % str(e)


if __name__ == "__main__":
    send_email()
